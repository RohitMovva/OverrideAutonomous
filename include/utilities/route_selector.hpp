#pragma once

#include "api.h"
#include "liblvgl/lvgl.h"
#include "navigation/trajectory.hpp"

#include <map>
#include <string>
#include <vector>

/**
 * @brief Route picker on the brain touch screen.
 *
 * A full-screen LVGL page, built on LVGL's task right after initialize() and kept
 * live in every competition state (LVGL's own task services touch input whether the robot is
 * disabled, in autonomous or in driver control). It lists the trajectory files in
 * /usd/routes one folder deep: the root view shows the subfolders ("suite",
 * "archive") followed by any loose routes, a folder view shows a back row and that
 * folder's routes. Tap a row or use the up/down buttons to pick one, "Rescan"
 * re-lists the card after a fresh export. The chosen route is loaded into the
 * shared Trajectory ahead of time so autonomous() starts without touching the card,
 * and the choice ("suite/corner_R6") is written to /usd/route_select.txt and
 * restored on the next boot, so a reflash is never needed to change routes.
 *
 * The brain cannot list directories, so the planner writes /usd/routes/folders.txt
 * (one subfolder name per line) whenever it exports; that index is what defines
 * the folders here.
 *
 * Threading, because LVGL is not thread-safe and the control loop must never wait
 * on the screen:
 *  - widget callbacks and a 100 ms lv_timer run on LVGL's task and are the ONLY
 *    code that touches widgets, including building the page on the first tick
 *    (PROS builds LVGL with LV_OS_NONE, so lv_lock() cannot guard a user task);
 *  - a low-priority loader task does every file operation (listing, loading,
 *    persisting) and never calls LVGL;
 *  - the two meet through a small state block under a pros::Mutex, which the LVGL
 *    side only try-locks so a load in progress can never stall the display.
 * autonomous() brackets its run with prepareForRun()/finishRun(); a selection made
 * mid-run is loaded once the run ends. opcontrol() hides the page so the LLEMU
 * debug lines underneath stay readable, and disabled() shows it again.
 */
class RouteSelector
{
public:
    RouteSelector(Trajectory &trajectory,
                  std::string routes_dir = "/usd/routes",
                  std::string persist_path = "/usd/route_select.txt");

    /// Scan the card, restore the saved choice (or fall back to `default_name`),
    /// start the LVGL timer that builds the page, load the route and start the
    /// loader task. Call after pros::lcd::initialize().
    void initialize(const std::string &default_name);

    /// Move the selection to the next/previous route in the open folder (wraps).
    /// Safe from any task.
    void next();
    void previous();

    /// Open a folder ("" for the root). Safe from any task.
    void enterFolder(const std::string &folder);

    /// Re-list the card and reload the selection. Safe from any task.
    void rescan();

    /// Show or hide the page. Safe from any task; applied on the next LVGL tick.
    void show();
    void hide();

    /// Make sure the selected route is the loaded one and hold the loader off
    /// until finishRun(). Returns false if nothing is loaded.
    bool prepareForRun();
    void finishRun();

    std::string selected() const;
    std::string loaded() const;

private:
    /// One row of the list: a folder, the back row, or a route.
    struct Entry
    {
        std::string label;
        std::string id;     ///< route name incl. folder, e.g. "suite/corner_R6"; empty for folders
        std::string folder; ///< folder to open when tapped (folders and the back row)
        bool is_route = false;
    };

    /// What the card holds: subfolders from folders.txt and the routes in each
    /// (key "" is the root).
    struct CardIndex
    {
        std::vector<std::string> folders;
        std::map<std::string, std::vector<std::string>> routes;
        bool ok = false;
    };

    // --- loader task side (file I/O, no LVGL) ---
    static void loaderTask(void *self);
    void loaderTick();
    CardIndex listCard();
    std::vector<std::string> listFolder(const std::string &folder, bool quiet = false);
    void loadSelected();
    void persistChoice(const std::string &name);
    void selectId(const std::string &id);
    void rebuildView();
    void stepSelection(int direction);

    // --- LVGL task side (widgets only) ---
    static void lvglTick(lv_timer_t *timer);
    static void onRowClicked(lv_event_t *event);
    static void onUpClicked(lv_event_t *event);
    static void onDownClicked(lv_event_t *event);
    static void onRescanClicked(lv_event_t *event);
    void buildPage();
    void rebuildList();
    void refreshWidgets();

    Trajectory &trajectory_;
    const std::string routes_dir_;
    const std::string persist_path_;

    // Shared state, guarded by mutex_.
    mutable pros::Mutex mutex_;
    CardIndex card_;
    std::string open_folder_;       ///< "" = root view
    std::vector<Entry> view_;       ///< rows of the open folder
    std::size_t index_ = 0;         ///< row of selected_ in view_, or view_.size() if not shown
    std::string selected_;          ///< route id
    std::string loaded_;
    std::string load_failed_;       ///< route whose load failed; not retried until the selection changes
    std::string status_;
    bool running_ = false;          ///< a run is in progress; the trajectory is read-only
    bool loading_ = false;
    bool visible_ = true;
    bool rescan_requested_ = false; ///< loader: re-list the card
    bool reload_requested_ = false; ///< loader: reload even if the name matches
    bool persist_requested_ = false;///< loader: write the choice to the card
    bool list_dirty_ = true;        ///< LVGL: rebuild the row widgets
    bool scroll_pending_ = true;    ///< LVGL: scroll the selected row into view

    // Widgets, touched only from LVGL's task.
    lv_obj_t *page_ = nullptr;
    lv_obj_t *list_ = nullptr;
    lv_obj_t *name_label_ = nullptr;
    lv_obj_t *status_label_ = nullptr;
    lv_obj_t *count_label_ = nullptr;
    std::vector<lv_obj_t *> rows_;
    lv_timer_t *timer_ = nullptr;
    pros::Task *loader_ = nullptr;
};
