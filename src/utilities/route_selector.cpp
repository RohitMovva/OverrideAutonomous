#include "utilities/route_selector.hpp"

#include "utilities/logger.hpp"
#include "utilities/sd_lock.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <memory>

namespace
{
constexpr std::uint32_t LOADER_PERIOD_MS = 100;
constexpr std::uint32_t LVGL_PERIOD_MS = 100;
constexpr std::uint32_t TRY_LOCK_MS = 5;
constexpr std::size_t LIST_BUFFER = 4096;
constexpr const char *FOLDER_INDEX = "folders.txt";
// Folder names to try when folders.txt is missing or empty (the planner writes it,
// but a card pulled before the write flushed can leave it blank).
constexpr const char *PROBE_FOLDERS[] = {"suite", "archive"};

constexpr int SCREEN_W = 480;
constexpr int SCREEN_H = 240;
constexpr int LIST_W = 290;
constexpr int PANEL_X = LIST_W + 8;
constexpr int PANEL_W = SCREEN_W - PANEL_X - 4;

constexpr std::uint32_t COLOR_BG = 0x101820;
constexpr std::uint32_t COLOR_PANEL = 0x1c2733;
constexpr std::uint32_t COLOR_ROW = 0x243241;
constexpr std::uint32_t COLOR_FOLDER = 0x2f3f52;
constexpr std::uint32_t COLOR_SELECTED = 0x1f7ae0;
constexpr std::uint32_t COLOR_TEXT = 0xf2f4f7;
constexpr std::uint32_t COLOR_MUTED = 0x9aa5b1;
constexpr std::uint32_t COLOR_OK = 0x3ddc84;
constexpr std::uint32_t COLOR_WARN = 0xffb347;
constexpr std::uint32_t COLOR_BAD = 0xff5c5c;

std::string trimmed(std::string s)
{
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' '))
    {
        s.pop_back();
    }
    return s;
}

std::string folderOf(const std::string &id)
{
    const auto slash = id.find('/');
    return slash == std::string::npos ? "" : id.substr(0, slash);
}

std::string baseOf(const std::string &id)
{
    const auto slash = id.find('/');
    return slash == std::string::npos ? id : id.substr(slash + 1);
}
} // namespace

RouteSelector::RouteSelector(Trajectory &trajectory, std::string routes_dir, std::string persist_path)
    : trajectory_(trajectory), routes_dir_(std::move(routes_dir)), persist_path_(std::move(persist_path))
{
}

// ---------------------------------------------------------------- lifecycle

void RouteSelector::initialize(const std::string &default_name)
{
    // Boot checkpoints. Each one is flushed to the card before the next step
    // starts, so a hang inside initialize() leaves the last completed step on
    // disk. Boots 0141/0143/0144 (09-19-2026) stopped after the card listing
    // with the brain still on the plain PROS screen and nothing else in the log,
    // which left the persist read, the view rebuild and the LVGL page build all
    // equally suspect. Never call this while holding an SdLock: flush() waits on
    // the logger task, which cannot write until the card is free.
    const auto checkpoint = [](const char *step)
    {
        Logger::getInstance()->log("RouteSelector: checkpoint %s", step);
        Logger::getInstance()->flush();
    };

    card_ = listCard();
    checkpoint("card listed");

    std::string wanted = default_name;
    std::string line;
    {
        SdLock lock;
        std::ifstream saved(persist_path_);
        if (saved.is_open() && std::getline(saved, line) && !trimmed(line).empty())
        {
            wanted = trimmed(line);
        }
    }
    checkpoint("saved choice read");

    // Does the saved/default id exist? Look in its folder (root if none).
    bool found = false;
    const auto in_folder = card_.routes.find(folderOf(wanted));
    if (in_folder != card_.routes.end())
    {
        found = std::find(in_folder->second.begin(), in_folder->second.end(), baseOf(wanted)) !=
                in_folder->second.end();
    }
    if (!found)
    {
        // First route anywhere: root first, then the folders in order.
        std::string first;
        if (!card_.routes[""].empty())
        {
            first = card_.routes[""].front();
        }
        for (const std::string &folder : card_.folders)
        {
            if (first.empty() && !card_.routes[folder].empty())
            {
                first = folder + "/" + card_.routes[folder].front();
            }
        }
        if (first.empty())
        {
            // No card or nothing listed: keep the default so the load attempt and
            // its error land in the log the way the hardcoded version's did.
            card_.routes[folderOf(wanted)].push_back(baseOf(wanted));
            first = wanted;
        }
        else
        {
            Logger::getInstance()->log("RouteSelector: '%s' not on the card, using '%s'",
                                       wanted.c_str(), first.c_str());
        }
        wanted = first;
    }
    selected_ = wanted;
    open_folder_ = folderOf(selected_);
    if (!card_.ok)
    {
        status_ = "no routes on card";
    }
    rebuildView();
    checkpoint("view rebuilt");

    // The page itself is built on the timer's first tick, on LVGL's task. No LVGL
    // call from a user task is safe: PROS builds LVGL with LV_OS_NONE, so lv_lock()
    // is a no-op and nothing stops the display task redrawing mid-call. Creating
    // the timer is the one exception left, and it is a single short list insert.
    timer_ = lv_timer_create(lvglTick, LVGL_PERIOD_MS, this);
    checkpoint("timer created");

    // First load is synchronous so initialize() still ends with a route in memory.
    loadSelected();

    loader_ = new pros::Task(loaderTask, this, TASK_PRIORITY_DEFAULT - 1, TASK_STACK_DEPTH_DEFAULT,
                             "route_selector");
    checkpoint("loader task started");
}

// ---------------------------------------------------------------- public, any task

void RouteSelector::next()
{
    stepSelection(+1);
}

void RouteSelector::previous()
{
    stepSelection(-1);
}

void RouteSelector::enterFolder(const std::string &folder)
{
    mutex_.take();
    open_folder_ = folder;
    rebuildView();
    scroll_pending_ = true;
    mutex_.give();
}

void RouteSelector::rescan()
{
    mutex_.take();
    rescan_requested_ = true;
    status_ = "rescanning card";
    mutex_.give();
}

void RouteSelector::show()
{
    mutex_.take();
    visible_ = true;
    mutex_.give();
}

void RouteSelector::hide()
{
    mutex_.take();
    visible_ = false;
    mutex_.give();
}

bool RouteSelector::prepareForRun()
{
    mutex_.take();
    const bool needs_load = loaded_ != selected_ || reload_requested_;
    mutex_.give();
    if (needs_load)
    {
        loadSelected();
    }
    mutex_.take();
    running_ = true;
    const bool ready = !loaded_.empty() && !trajectory_.empty();
    status_ = ready ? "RUNNING " + loaded_ : "NO ROUTE LOADED";
    mutex_.give();
    if (!ready)
    {
        Logger::getInstance()->logError("RouteSelector: no route loaded, cannot run");
    }
    return ready;
}

void RouteSelector::finishRun()
{
    mutex_.take();
    running_ = false;
    if (!loaded_.empty())
    {
        status_ = "loaded " + std::to_string(trajectory_.size()) + " pts";
    }
    mutex_.give();
}

std::string RouteSelector::selected() const
{
    mutex_.take();
    std::string s = selected_;
    mutex_.give();
    return s;
}

std::string RouteSelector::loaded() const
{
    mutex_.take();
    std::string s = loaded_;
    mutex_.give();
    return s;
}

// ---------------------------------------------------------------- state (under mutex_)

void RouteSelector::rebuildView()
{
    // Caller holds mutex_ (or is initialize(), before any other task exists).
    view_.clear();
    if (open_folder_.empty())
    {
        for (const std::string &folder : card_.folders)
        {
            Entry e;
            e.label = std::string(LV_SYMBOL_DIRECTORY) + "  " + folder;
            e.folder = folder;
            view_.push_back(e);
        }
    }
    else
    {
        Entry back;
        back.label = std::string(LV_SYMBOL_LEFT) + "  back";
        back.folder = "";
        view_.push_back(back);
    }
    for (const std::string &name : card_.routes[open_folder_])
    {
        Entry e;
        e.label = name;
        e.id = open_folder_.empty() ? name : open_folder_ + "/" + name;
        e.is_route = true;
        view_.push_back(e);
    }
    index_ = view_.size();
    for (std::size_t i = 0; i < view_.size(); ++i)
    {
        if (view_[i].is_route && view_[i].id == selected_)
        {
            index_ = i;
        }
    }
    list_dirty_ = true;
}

void RouteSelector::selectId(const std::string &id)
{
    mutex_.take();
    selected_ = id;
    load_failed_.clear();
    persist_requested_ = true;
    scroll_pending_ = true;
    index_ = view_.size();
    for (std::size_t i = 0; i < view_.size(); ++i)
    {
        if (view_[i].is_route && view_[i].id == selected_)
        {
            index_ = i;
        }
    }
    Logger::getInstance()->log("RouteSelector: selected '%s'", selected_.c_str());
    mutex_.give();
}

void RouteSelector::stepSelection(int direction)
{
    mutex_.take();
    std::vector<std::size_t> route_rows;
    for (std::size_t i = 0; i < view_.size(); ++i)
    {
        if (view_[i].is_route)
        {
            route_rows.push_back(i);
        }
    }
    std::string target;
    if (!route_rows.empty())
    {
        std::size_t pos = 0;
        bool shown = false;
        for (std::size_t k = 0; k < route_rows.size(); ++k)
        {
            if (route_rows[k] == index_)
            {
                pos = k;
                shown = true;
            }
        }
        // If the selection is in another folder, the first step lands on the first
        // (or last) route of the open one instead of skipping past it.
        if (shown)
        {
            pos = (pos + route_rows.size() + (direction > 0 ? 1 : route_rows.size() - 1)) % route_rows.size();
        }
        else if (direction < 0)
        {
            pos = route_rows.size() - 1;
        }
        target = view_[route_rows[pos]].id;
    }
    mutex_.give();
    if (!target.empty())
    {
        selectId(target);
    }
}

// ---------------------------------------------------------------- loader task

void RouteSelector::loaderTask(void *self)
{
    auto *selector = static_cast<RouteSelector *>(self);
    while (true)
    {
        selector->loaderTick();
        pros::delay(LOADER_PERIOD_MS);
    }
}

void RouteSelector::loaderTick()
{
    mutex_.take();
    const bool do_rescan = rescan_requested_;
    const bool do_persist = persist_requested_;
    const std::string name = selected_;
    rescan_requested_ = false;
    persist_requested_ = false;
    // A route that failed to load is not retried every tick: only a new selection
    // or a rescan clears load_failed_.
    const bool do_load = !running_ && (reload_requested_ ||
                                       (loaded_ != selected_ && selected_ != load_failed_));
    mutex_.give();

    if (do_persist)
    {
        persistChoice(name);
    }

    if (do_rescan)
    {
        CardIndex fresh = listCard();
        mutex_.take();
        if (fresh.ok)
        {
            card_ = fresh;
            if (card_.routes.find(open_folder_) == card_.routes.end())
            {
                open_folder_.clear();
            }
            rebuildView();
            reload_requested_ = true;
            load_failed_.clear();
            std::size_t total = 0;
            for (const auto &kv : card_.routes)
            {
                total += kv.second.size();
            }
            status_ = std::to_string(total) + " routes in " + std::to_string(card_.folders.size()) + " folders";
        }
        else
        {
            status_ = "card list failed";
        }
        scroll_pending_ = true;
        mutex_.give();
        return; // the load happens on the next tick, once the list has settled
    }

    if (do_load)
    {
        loadSelected();
    }
}

std::vector<std::string> RouteSelector::listFolder(const std::string &folder, bool quiet)
{
    // pros::usd::list_files takes the path relative to the card root and returns
    // newline-separated file names (directories are not included).
    std::string relative = routes_dir_;
    const std::string usd_prefix = "/usd";
    if (relative.rfind(usd_prefix, 0) == 0)
    {
        relative = relative.substr(usd_prefix.size());
    }
    if (!folder.empty())
    {
        relative += "/" + folder;
    }
    if (relative.empty())
    {
        relative = "/";
    }

    std::vector<std::string> names;
    std::unique_ptr<char[]> buffer(new char[LIST_BUFFER]);
    std::memset(buffer.get(), 0, LIST_BUFFER);
    int32_t rc;
    {
        SdLock lock;
        rc = pros::usd::list_files(relative.c_str(), buffer.get(), LIST_BUFFER - 1);
    }
    if (rc == PROS_ERR)
    {
        if (!quiet)
        {
            Logger::getInstance()->logError("RouteSelector: cannot list %s (errno %d)", relative.c_str(), errno);
        }
        return names;
    }

    const std::string ext = ".txt";
    const char *p = buffer.get();
    while (*p)
    {
        const char *end = std::strchr(p, '\n');
        std::string name = trimmed(std::string(p, end ? static_cast<std::size_t>(end - p) : std::strlen(p)));
        if (name.size() > ext.size() && name.compare(name.size() - ext.size(), ext.size(), ext) == 0 &&
            name != FOLDER_INDEX)
        {
            names.push_back(name.substr(0, name.size() - ext.size()));
        }
        if (!end)
        {
            break;
        }
        p = end + 1;
    }
    std::sort(names.begin(), names.end());
    return names;
}

RouteSelector::CardIndex RouteSelector::listCard()
{
    CardIndex index;

    {
        // Scoped, not held across the listFolder() calls below: those take the
        // card lock themselves and it is not recursive.
        SdLock lock;
        std::ifstream folders(routes_dir_ + "/" + FOLDER_INDEX);
        std::string line;
        while (folders.is_open() && std::getline(folders, line))
        {
            line = trimmed(line);
            if (!line.empty() && line.front() != '#')
            {
                index.folders.push_back(line);
            }
        }
    }
    std::sort(index.folders.begin(), index.folders.end());

    if (index.folders.empty())
    {
        // No usable index: probe the conventional folders so the card still works.
        for (const char *name : PROBE_FOLDERS)
        {
            if (!listFolder(name, true).empty())
            {
                index.folders.push_back(name);
            }
        }
        if (!index.folders.empty())
        {
            Logger::getInstance()->logWarning(
                "RouteSelector: %s missing or empty; found %d folders by probing",
                FOLDER_INDEX, static_cast<int>(index.folders.size()));
        }
    }

    index.routes[""] = listFolder("");
    index.ok = !index.routes[""].empty() || !index.folders.empty();
    std::size_t total = index.routes[""].size();
    for (const std::string &folder : index.folders)
    {
        index.routes[folder] = listFolder(folder);
        total += index.routes[folder].size();
        index.ok = index.ok || !index.routes[folder].empty();
    }
    Logger::getInstance()->log("RouteSelector: %d routes on the card in %d folders",
                               static_cast<int>(total), static_cast<int>(index.folders.size()));
    return index;
}

void RouteSelector::loadSelected()
{
    mutex_.take();
    if (running_)
    {
        mutex_.give();
        return;
    }
    const std::string name = selected_;
    reload_requested_ = false;
    loading_ = true;
    status_ = "loading " + name;
    mutex_.give();

    // The trajectory is only ever read while running_, and running_ is false here,
    // so the file read can proceed without holding the mutex against the display.
    trajectory_.loadFromFile(routes_dir_ + "/" + name + ".txt");

    mutex_.take();
    loading_ = false;
    if (trajectory_.empty())
    {
        loaded_.clear();
        load_failed_ = name;
        status_ = "NOT FOUND on card";
        Logger::getInstance()->logError("RouteSelector: route '%s' missing or empty", name.c_str());
    }
    else
    {
        loaded_ = name;
        load_failed_.clear();
        status_ = "loaded " + std::to_string(trajectory_.size()) + " pts";
        Logger::getInstance()->log("init: trajectory '%s' loaded size=%d", name.c_str(),
                                   static_cast<int>(trajectory_.size()));
    }
    mutex_.give();
}

void RouteSelector::persistChoice(const std::string &name)
{
    SdLock lock;
    std::ofstream out(persist_path_, std::ios::trunc);
    if (!out.is_open())
    {
        Logger::getInstance()->logError("RouteSelector: cannot write %s", persist_path_.c_str());
        return;
    }
    out << name << '\n';
}

// ---------------------------------------------------------------- LVGL task

void RouteSelector::onRowClicked(lv_event_t *event)
{
    auto *self = static_cast<RouteSelector *>(lv_event_get_user_data(event));
    lv_obj_t *row = static_cast<lv_obj_t *>(lv_event_get_target(event));
    const std::size_t i = static_cast<std::size_t>(lv_obj_get_index(row));

    self->mutex_.take();
    const Entry entry = i < self->view_.size() ? self->view_[i] : Entry{};
    self->mutex_.give();

    if (entry.is_route)
    {
        self->selectId(entry.id);
    }
    else if (!entry.label.empty())
    {
        self->enterFolder(entry.folder);
    }
}

void RouteSelector::onUpClicked(lv_event_t *event)
{
    static_cast<RouteSelector *>(lv_event_get_user_data(event))->previous();
}

void RouteSelector::onDownClicked(lv_event_t *event)
{
    static_cast<RouteSelector *>(lv_event_get_user_data(event))->next();
}

void RouteSelector::onRescanClicked(lv_event_t *event)
{
    static_cast<RouteSelector *>(lv_event_get_user_data(event))->rescan();
}

void RouteSelector::buildPage()
{
    // The brain sat on the plain PROS screen through the 09-19-2026 hangs, so the
    // page root either was never created or never got drawn. This checkpoint is
    // what tells those two apart: it lands after the first LVGL call returns.
    page_ = lv_obj_create(lv_screen_active());
    Logger::getInstance()->log("RouteSelector: checkpoint page root created");
    Logger::getInstance()->flush();
    lv_obj_set_size(page_, SCREEN_W, SCREEN_H);
    lv_obj_set_pos(page_, 0, 0);
    lv_obj_set_style_bg_color(page_, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_border_width(page_, 0, 0);
    lv_obj_set_style_radius(page_, 0, 0);
    lv_obj_set_style_pad_all(page_, 4, 0);
    lv_obj_remove_flag(page_, LV_OBJ_FLAG_SCROLLABLE);

    // Route list, left.
    list_ = lv_list_create(page_);
    lv_obj_set_size(list_, LIST_W, SCREEN_H - 8);
    lv_obj_set_pos(list_, 0, 0);
    lv_obj_set_style_bg_color(list_, lv_color_hex(COLOR_PANEL), 0);
    lv_obj_set_style_border_width(list_, 0, 0);
    lv_obj_set_style_pad_row(list_, 2, 0);

    // Panel, right: selection, status, controls.
    lv_obj_t *panel = lv_obj_create(page_);
    lv_obj_set_size(panel, PANEL_W, SCREEN_H - 8);
    lv_obj_set_pos(panel, PANEL_X, 0);
    lv_obj_set_style_bg_color(panel, lv_color_hex(COLOR_PANEL), 0);
    lv_obj_set_style_border_width(panel, 0, 0);
    lv_obj_set_style_pad_all(panel, 6, 0);
    lv_obj_remove_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(panel);
    lv_label_set_text(title, "ROUTE");
    lv_obj_set_style_text_color(title, lv_color_hex(COLOR_MUTED), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_12, 0);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 0, 0);

    name_label_ = lv_label_create(panel);
    lv_label_set_long_mode(name_label_, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(name_label_, PANEL_W - 12);
    lv_obj_set_style_text_color(name_label_, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_set_style_text_font(name_label_, &lv_font_montserrat_20, 0);
    lv_obj_align(name_label_, LV_ALIGN_TOP_LEFT, 0, 16);

    status_label_ = lv_label_create(panel);
    lv_label_set_long_mode(status_label_, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(status_label_, PANEL_W - 12);
    lv_obj_set_style_text_color(status_label_, lv_color_hex(COLOR_MUTED), 0);
    lv_obj_set_style_text_font(status_label_, &lv_font_montserrat_14, 0);
    lv_obj_align(status_label_, LV_ALIGN_TOP_LEFT, 0, 66);

    auto make_button = [&](const char *text, lv_event_cb_t cb, int x, int y, int w, int h) {
        lv_obj_t *button = lv_button_create(panel);
        lv_obj_set_size(button, w, h);
        lv_obj_set_pos(button, x, y);
        lv_obj_set_style_bg_color(button, lv_color_hex(COLOR_ROW), 0);
        lv_obj_set_style_bg_color(button, lv_color_hex(COLOR_SELECTED), LV_STATE_PRESSED);
        lv_obj_add_event_cb(button, cb, LV_EVENT_CLICKED, this);
        lv_obj_t *label = lv_label_create(button);
        lv_label_set_text(label, text);
        lv_obj_set_style_text_color(label, lv_color_hex(COLOR_TEXT), 0);
        lv_obj_set_style_text_font(label, &lv_font_montserrat_18, 0);
        lv_obj_center(label);
        return button;
    };
    const int button_h = 46;
    const int button_y = SCREEN_H - 8 - 12 - button_h - 6 - button_h;
    const int half = (PANEL_W - 12 - 6) / 2;
    make_button(LV_SYMBOL_UP, onUpClicked, 0, button_y, half, button_h);
    make_button(LV_SYMBOL_DOWN, onDownClicked, half + 6, button_y, half, button_h);
    make_button(LV_SYMBOL_REFRESH "  Rescan card", onRescanClicked, 0, button_y + button_h + 6,
                PANEL_W - 12, button_h);

    count_label_ = lv_label_create(panel);
    lv_obj_set_style_text_color(count_label_, lv_color_hex(COLOR_MUTED), 0);
    lv_obj_set_style_text_font(count_label_, &lv_font_montserrat_12, 0);
    lv_obj_align(count_label_, LV_ALIGN_TOP_LEFT, 0, button_y - 16);

    rebuildList();
    Logger::getInstance()->log("RouteSelector: checkpoint rows built (%d)",
                               static_cast<int>(rows_.size()));
    Logger::getInstance()->flush();
    refreshWidgets();
}

void RouteSelector::rebuildList()
{
    lv_obj_clean(list_);
    rows_.clear();
    mutex_.take();
    const std::vector<Entry> entries = view_;
    mutex_.give();
    for (const Entry &entry : entries)
    {
        lv_obj_t *row = lv_list_add_button(list_, nullptr, entry.label.c_str());
        lv_obj_set_height(row, 40);
        lv_obj_set_style_bg_color(row, lv_color_hex(entry.is_route ? COLOR_ROW : COLOR_FOLDER), 0);
        lv_obj_set_style_bg_opa(row, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(row, lv_color_hex(COLOR_SELECTED), LV_STATE_CHECKED);
        lv_obj_set_style_text_color(row, lv_color_hex(COLOR_TEXT), 0);
        lv_obj_set_style_text_font(row, &lv_font_montserrat_16, 0);
        lv_obj_set_style_radius(row, 4, 0);
        lv_obj_add_event_cb(row, onRowClicked, LV_EVENT_CLICKED, this);
        rows_.push_back(row);
    }
}

void RouteSelector::refreshWidgets()
{
    if (!mutex_.take(TRY_LOCK_MS))
    {
        return; // a load is in progress; try again next tick
    }
    const bool visible = visible_;
    const bool rebuild = list_dirty_;
    const bool scroll = scroll_pending_;
    const std::size_t index = index_;
    const std::string name = selected_;
    const std::string status = status_;
    const std::string folder = open_folder_;
    std::size_t routes_here = 0;
    for (const Entry &e : view_)
    {
        routes_here += e.is_route ? 1 : 0;
    }
    const bool loaded_matches = loaded_ == selected_ && !loaded_.empty();
    const bool running = running_;
    const bool loading = loading_;
    list_dirty_ = false;
    scroll_pending_ = false;
    mutex_.give();

    if (visible)
    {
        lv_obj_remove_flag(page_, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(page_, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    if (rebuild)
    {
        rebuildList();
    }

    for (std::size_t i = 0; i < rows_.size(); ++i)
    {
        if (i == index)
        {
            lv_obj_add_state(rows_[i], LV_STATE_CHECKED);
        }
        else
        {
            lv_obj_remove_state(rows_[i], LV_STATE_CHECKED);
        }
    }
    if (scroll && !rows_.empty())
    {
        lv_obj_scroll_to_view(rows_[index < rows_.size() ? index : 0], LV_ANIM_OFF);
    }

    lv_label_set_text(name_label_, name.c_str());
    lv_label_set_text(status_label_, status.c_str());
    const std::uint32_t colour = running ? COLOR_WARN : loading ? COLOR_MUTED : loaded_matches ? COLOR_OK : COLOR_BAD;
    lv_obj_set_style_text_color(status_label_, lv_color_hex(colour), 0);
    lv_label_set_text_fmt(count_label_, "%s: %d routes", folder.empty() ? "root" : folder.c_str(),
                          static_cast<int>(routes_here));
}

void RouteSelector::lvglTick(lv_timer_t *timer)
{
    auto *self = static_cast<RouteSelector *>(lv_timer_get_user_data(timer));
    if (self->page_ == nullptr)
    {
        self->buildPage(); // ends with a refreshWidgets()
        Logger::getInstance()->log("RouteSelector: checkpoint page built");
        return;
    }
    self->refreshWidgets();
}
