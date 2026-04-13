#include "screen_manager.hpp"
#include "telemetry_formatting.hpp"
#include <cstdio>
#include "abclib/sysid/pid_tuner.hpp"

namespace abclib::ui
{

    void ScreenManager::initialize(DefaultScreen default_screen)
    {
        // Create the main telemetry tabview
        tabview = lv_tabview_create(lv_screen_active());

        // Hide the default tab bar (we'll use custom navigation)
        lv_tabview_set_tab_bar_size(tabview, 0);

        // Create all telemetry tabs
        tab_overview = lv_tabview_add_tab(tabview, "Overview");
        tab_pid = lv_tabview_add_tab(tabview, "PID");
        tab_trajectory = lv_tabview_add_tab(tabview, "Trajectory");
        tab_performance = lv_tabview_add_tab(tabview, "Performance");
        tab_blended_debug = lv_tabview_add_tab(tabview, "EKF");
        tab_config = lv_tabview_add_tab(tabview, "Config");

        // Setup all telemetry tabs
        create_overview_tab();
        create_pid_tab();
        create_trajectory_tab();
        create_performance_tab();
        create_blended_debug_tab();
        create_config_tab();

        // Create custom navigation bar
        create_navigation_bar();

        // Create full-screen image overlay (not a tab anymore)
        create_fullscreen_image();
        create_calibration_screen();
        // Create autonomous screen (initially hidden)
        create_autonomous_screen();

        // PUT IT HERE AT THE END - Set default screen
        current_screen_index = static_cast<int>(default_screen);
        lv_tabview_set_active(tabview, current_screen_index, LV_ANIM_OFF);
        update_current_screen_label();
        update_navigation_buttons();
    }

    void ScreenManager::create_navigation_bar()
    {
        // Create container for navigation bar at the top
        nav_bar = lv_obj_create(lv_screen_active());
        lv_obj_set_size(nav_bar, LV_PCT(100), 50);
        lv_obj_align(nav_bar, LV_ALIGN_TOP_MID, 0, 0);
        lv_obj_set_style_pad_all(nav_bar, 5, 0);
        lv_obj_remove_flag(nav_bar, LV_OBJ_FLAG_SCROLLABLE);

        // Left arrow button
        btn_prev = lv_button_create(nav_bar);
        lv_obj_set_size(btn_prev, 40, 40);
        lv_obj_align(btn_prev, LV_ALIGN_LEFT_MID, 0, 0);
        lv_obj_t *label_prev = lv_label_create(btn_prev);
        lv_label_set_text(label_prev, LV_SYMBOL_LEFT);
        lv_obj_center(label_prev);
        lv_obj_add_event_cb(btn_prev, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->navigate_prev(); }, LV_EVENT_CLICKED, this);

        // Auton selector button (now enabled)
        btn_auton = lv_button_create(nav_bar);
        lv_obj_set_size(btn_auton, 80, 40);
        lv_obj_align(btn_auton, LV_ALIGN_LEFT_MID, 45, 0);
        lv_obj_t *label_auton = lv_label_create(btn_auton);
        lv_label_set_text(label_auton, "Auton");
        lv_obj_center(label_auton);
        lv_obj_add_event_cb(btn_auton, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->show_autonomous_screen(); }, LV_EVENT_CLICKED, this);

        // Current screen name label (center)
        label_current_screen = lv_label_create(nav_bar);
        lv_label_set_text(label_current_screen, "Overview");
        lv_obj_align(label_current_screen, LV_ALIGN_CENTER, 0, 0);

        // Image button
        btn_image = lv_button_create(nav_bar);
        lv_obj_set_size(btn_image, 80, 40);
        lv_obj_align(btn_image, LV_ALIGN_RIGHT_MID, -45, 0);
        lv_obj_t *label_image = lv_label_create(btn_image);
        lv_label_set_text(label_image, "Image");
        lv_obj_center(label_image);
        lv_obj_add_event_cb(btn_image, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->show_fullscreen_image(); }, LV_EVENT_CLICKED, this);

        // Right arrow button
        btn_next = lv_button_create(nav_bar);
        lv_obj_set_size(btn_next, 40, 40);
        lv_obj_align(btn_next, LV_ALIGN_RIGHT_MID, 0, 0);
        lv_obj_t *label_next = lv_label_create(btn_next);
        lv_label_set_text(label_next, LV_SYMBOL_RIGHT);
        lv_obj_center(label_next);
        lv_obj_add_event_cb(btn_next, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->navigate_next(); }, LV_EVENT_CLICKED, this);

        // Update button states for initial screen
        update_navigation_buttons();
    }

    void ScreenManager::create_autonomous_screen()
    {
        // Create the autonomous tabview (initially hidden)
        auton_tabview = lv_tabview_create(lv_screen_active());

        // Position it to cover the entire screen
        lv_obj_set_size(auton_tabview, LV_PCT(100), LV_PCT(100));
        lv_obj_align(auton_tabview, LV_ALIGN_TOP_LEFT, 0, 0);

        // Create all 6 tabs
        auton_tab_red = lv_tabview_add_tab(auton_tabview, "Red");
        auton_tab_blue = lv_tabview_add_tab(auton_tabview, "Blue");
        auton_tab_skills = lv_tabview_add_tab(auton_tabview, "Skills");
        auton_tab_test = lv_tabview_add_tab(auton_tabview, "Test");
        auton_tab_telemetry = lv_tabview_add_tab(auton_tabview, "Telemetry");
        auton_tab_image = lv_tabview_add_tab(auton_tabview, "Image");

        // Setup all autonomous tabs
        create_auton_red_tab();
        create_auton_blue_tab();
        create_auton_skills_tab();
        create_auton_test_tab();
        create_auton_telemetry_tab();
        create_auton_image_tab();

        // Add event callback to detect tab changes
        lv_obj_add_event_cb(auton_tabview, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        lv_obj_t* tabview = (lv_obj_t*)lv_event_get_target(e);
        uint32_t active_tab = lv_tabview_get_tab_active(tabview);
        
        // Tab 4 is Telemetry - switch back to main telemetry screen
        if (active_tab == 4) {
            manager->show_telemetry_screen();
        }
        // Tab 5 is Image - show fullscreen image
        else if (active_tab == 5) {
            manager->show_fullscreen_image();
        } }, LV_EVENT_VALUE_CHANGED, this);

        // Hide initially
        lv_obj_add_flag(auton_tabview, LV_OBJ_FLAG_HIDDEN);
    }

    void ScreenManager::create_auton_red_tab()
    {
        // Title
        lv_obj_t *title = lv_label_create(auton_tab_red);
        lv_label_set_text(title, "RED ALLIANCE");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

        // Dropdown
        red_dropdown = lv_dropdown_create(auton_tab_red);
        lv_obj_set_size(red_dropdown, 300, 50);
        lv_obj_align(red_dropdown, LV_ALIGN_CENTER, 0, -10);

        // Store category in dropdown's user data
        static abclib::auton::AutonCategory red_category = abclib::auton::AutonCategory::RED;
        lv_obj_set_user_data(red_dropdown, &red_category);

        // Auto-populate and setup callback
        populate_dropdown(red_dropdown, abclib::auton::AutonCategory::RED);
        setup_dropdown_callback(red_dropdown, abclib::auton::AutonCategory::RED);

        // Confirm button
        confirm_btn_red = lv_button_create(auton_tab_red);
        lv_obj_set_size(confirm_btn_red, 280, 50);
        lv_obj_align(confirm_btn_red, LV_ALIGN_BOTTOM_MID, 0, -10);
        lv_obj_set_style_bg_color(confirm_btn_red, lv_color_hex(CONFIRM_COLOR_NORMAL), 0);
        lv_obj_t *label_confirm = lv_label_create(confirm_btn_red);
        lv_label_set_text(label_confirm, "CONFIRM");
        lv_obj_center(label_confirm);
        lv_obj_add_event_cb(confirm_btn_red, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->confirm_auton_selection(); }, LV_EVENT_CLICKED, this);
    }

    void ScreenManager::create_auton_blue_tab()
    {
        // Title
        lv_obj_t *title = lv_label_create(auton_tab_blue);
        lv_label_set_text(title, "BLUE ALLIANCE");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

        // Dropdown
        blue_dropdown = lv_dropdown_create(auton_tab_blue);
        lv_obj_set_size(blue_dropdown, 300, 50);
        lv_obj_align(blue_dropdown, LV_ALIGN_CENTER, 0, -10);

        // Store category in dropdown's user data
        static abclib::auton::AutonCategory blue_category = abclib::auton::AutonCategory::BLUE;
        lv_obj_set_user_data(blue_dropdown, &blue_category);

        // Auto-populate and setup callback
        populate_dropdown(blue_dropdown, abclib::auton::AutonCategory::BLUE);
        setup_dropdown_callback(blue_dropdown, abclib::auton::AutonCategory::BLUE);

        // Confirm button
        confirm_btn_blue = lv_button_create(auton_tab_blue);
        lv_obj_set_size(confirm_btn_blue, 280, 50);
        lv_obj_align(confirm_btn_blue, LV_ALIGN_BOTTOM_MID, 0, -10);
        lv_obj_set_style_bg_color(confirm_btn_blue, lv_color_hex(CONFIRM_COLOR_NORMAL), 0);
        lv_obj_t *label_confirm = lv_label_create(confirm_btn_blue);
        lv_label_set_text(label_confirm, "CONFIRM");
        lv_obj_center(label_confirm);
        lv_obj_add_event_cb(confirm_btn_blue, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->confirm_auton_selection(); }, LV_EVENT_CLICKED, this);
    }
    void ScreenManager::create_auton_skills_tab()
    {
        // Title
        lv_obj_t *title = lv_label_create(auton_tab_skills);
        lv_label_set_text(title, "SKILLS");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

        // Dropdown
        skills_dropdown = lv_dropdown_create(auton_tab_skills);
        lv_obj_set_size(skills_dropdown, 300, 50);
        lv_obj_align(skills_dropdown, LV_ALIGN_CENTER, 0, -10);

        // Store category in dropdown's user data
        static abclib::auton::AutonCategory skills_category = abclib::auton::AutonCategory::SKILLS;
        lv_obj_set_user_data(skills_dropdown, &skills_category);

        // Auto-populate and setup callback
        populate_dropdown(skills_dropdown, abclib::auton::AutonCategory::SKILLS);
        setup_dropdown_callback(skills_dropdown, abclib::auton::AutonCategory::SKILLS);

        // Confirm button
        confirm_btn_skills = lv_button_create(auton_tab_skills);
        lv_obj_set_size(confirm_btn_skills, 280, 50);
        lv_obj_align(confirm_btn_skills, LV_ALIGN_BOTTOM_MID, 0, -10);
        lv_obj_set_style_bg_color(confirm_btn_skills, lv_color_hex(CONFIRM_COLOR_NORMAL), 0);
        lv_obj_t *label_confirm = lv_label_create(confirm_btn_skills);
        lv_label_set_text(label_confirm, "CONFIRM");
        lv_obj_center(label_confirm);
        lv_obj_add_event_cb(confirm_btn_skills, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->confirm_auton_selection(); }, LV_EVENT_CLICKED, this);
    }
    void ScreenManager::create_blended_debug_tab()
    {
        // Create 5 full-width labels (same as overview style)
        for (int i = 0; i < 5; i++)
        {
            lv_obj_t *label = lv_label_create(tab_blended_debug);
            lv_label_set_text(label, "Loading...");
            lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 60 + i * 30);
            lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(label, 450); // Full width like overview
            blended_debug_labels.push_back(label);
        }
    }

    void ScreenManager::update_blended_debug_tab(const telemetry::TelemetryData &data)
    {
        if (blended_debug_labels.size() >= 5)
        {
            if (!data.data_valid)
            {
                lv_label_set_text(blended_debug_labels[0], "Waiting for telemetry...");
                lv_label_set_text(blended_debug_labels[1], "");
                lv_label_set_text(blended_debug_labels[2], "");
                lv_label_set_text(blended_debug_labels[3], "");
                lv_label_set_text(blended_debug_labels[4], "");
                return;
            }

            char buf[256];

            // Sensor labels (adjust based on your actual config)
            // TODO: Make this configurable or derive from sensor bearing angles
            const char *sensor_labels = "[F,L,B,R]";

            // Label 0: Sensor array + both poses + velocity + omega
            std::string pose_c = format_pose_tuple(
                data.pose_corner.x_inches(),
                data.pose_corner.y_inches(),
                data.pose_corner.theta_deg());
            std::string pose_s = format_pose_tuple(
                data.pose_standard.x_inches(),
                data.pose_standard.y_inches(),
                data.pose_standard.theta_deg());

            snprintf(buf, sizeof(buf), "%s C:%s S:%s V:%.1fips W:%.2f°/s",
                     sensor_labels,
                     pose_c.c_str(),
                     pose_s.c_str(),
                     data.pose_corner.v.to_ips(),
                     data.pose_corner.omega.to_deg_per_sec());
            lv_label_set_text(blended_debug_labels[0], buf);

            // Label 1: Measured and Expected distances
            std::vector<double> meas_mm, exp_mm;
            for (const auto &m : data.distance_measured)
            {
                meas_mm.push_back(m.to_mm());
            }
            for (const auto &e : data.distance_expected)
            {
                exp_mm.push_back(e.to_mm());
            }

            std::string meas_str = format_value_vector_always(meas_mm, "%.0f");
            std::string exp_str = format_value_vector_always(exp_mm, "%.0f");

            snprintf(buf, sizeof(buf), "Meas:%smm  Exp:%smm",
                     meas_str.c_str(),
                     exp_str.c_str());
            lv_label_set_text(blended_debug_labels[1], buf);

            // Label 2: Innovation + Mahalanobis + Wall + Valid
            std::vector<double> inn_mm;
            for (const auto &i : data.distance_innovation)
            {
                inn_mm.push_back(i.to_mm());
            }

            std::string inn_str = format_value_vector_always(inn_mm, "%.0f");
            std::string dm_str = format_value_vector_always(data.distance_mahalanobis, "%.1f");
            std::string wall_str = format_wall_vector_always(data.distance_walls);
            std::string valid_str = format_valid_vector(data.distance_valid);

            snprintf(buf, sizeof(buf), "Inn:%smm d_M:%s Wall:%s Valid:%s",
                     inn_str.c_str(),
                     dm_str.c_str(),
                     wall_str.c_str(),
                     valid_str.c_str());
            lv_label_set_text(blended_debug_labels[2], buf);

            // Label 3: show only the active blend mode
            if (data.kalman_blending_active)
            {
                std::string kalman_str = format_blend_vector(data.distance_kalman_gains);
                snprintf(buf, sizeof(buf), "K:%s Corr:%s Act:%d/%d",
                         kalman_str.c_str(),
                         format_correction(
                             data.correction_x_frame.to_inches(),
                             data.correction_y_frame.to_inches())
                             .c_str(),
                         data.num_active_sensors,
                         data.num_total_sensors);
            }
            else
            {
                std::string alpha_str = format_blend_vector(data.distance_blend_factors);
                snprintf(buf, sizeof(buf), "a:%s Corr:%s Act:%d/%d",
                         alpha_str.c_str(),
                         format_correction(
                             data.correction_x_frame.to_inches(),
                             data.correction_y_frame.to_inches())
                             .c_str(),
                         data.num_active_sensors,
                         data.num_total_sensors);
            }
            lv_label_set_text(blended_debug_labels[3], buf);

            // Label 4: Blend status + Safe + X/Y uncertainty + Total corrections
            std::string total_corr = format_correction(
                data.correction_x_total.to_inches(),
                data.correction_y_total.to_inches());

            char unc_buf[64];
            if (data.has_covariance)
            {
                snprintf(unc_buf, sizeof(unc_buf), "X:%.2f Y:%.2f Th:%.1f°",
                         data.x_uncertainty.to_inches(),
                         data.y_uncertainty.to_inches(),
                         data.heading_uncertainty.to_degrees());
            }
            else
            {
                snprintf(unc_buf, sizeof(unc_buf), "Unc:N/A");
            }

            snprintf(buf, sizeof(buf), "Blend:%s Safe:%s Unc:%s Tot:%s",
                     data.blending_enabled ? "ON" : "OFF",
                     data.blending_safe ? "YES" : "NO",
                     unc_buf,
                     total_corr.c_str());
            lv_label_set_text(blended_debug_labels[4], buf);
        }
    }
    void ScreenManager::create_auton_test_tab()
    {
        // Title
        lv_obj_t *title = lv_label_create(auton_tab_test);
        lv_label_set_text(title, "TEST ROUTINES");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

        // Dropdown
        test_dropdown = lv_dropdown_create(auton_tab_test);
        lv_obj_set_size(test_dropdown, 300, 50);
        lv_obj_align(test_dropdown, LV_ALIGN_CENTER, 0, -10);

        // Store category in dropdown's user data
        static abclib::auton::AutonCategory test_category = abclib::auton::AutonCategory::TEST;
        lv_obj_set_user_data(test_dropdown, &test_category);

        // Auto-populate and setup callback
        populate_dropdown(test_dropdown, abclib::auton::AutonCategory::TEST);
        setup_dropdown_callback(test_dropdown, abclib::auton::AutonCategory::TEST);

        // Confirm button
        confirm_btn_test = lv_button_create(auton_tab_test);
        lv_obj_set_size(confirm_btn_test, 280, 50);
        lv_obj_align(confirm_btn_test, LV_ALIGN_BOTTOM_MID, 0, -10);
        lv_obj_set_style_bg_color(confirm_btn_test, lv_color_hex(CONFIRM_COLOR_NORMAL), 0);
        lv_obj_t *label_confirm = lv_label_create(confirm_btn_test);
        lv_label_set_text(label_confirm, "CONFIRM");
        lv_obj_center(label_confirm);
        lv_obj_add_event_cb(confirm_btn_test, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->confirm_auton_selection(); }, LV_EVENT_CLICKED, this);
    }

    void ScreenManager::confirm_auton_selection()
    {
        // Get which button was clicked
        uint32_t active_tab = lv_tabview_get_tab_active(auton_tabview);
        lv_obj_t *clicked_btn = nullptr;

        switch (active_tab)
        {
        case 0:
            clicked_btn = confirm_btn_red;
            break;
        case 1:
            clicked_btn = confirm_btn_blue;
            break;
        case 2:
            clicked_btn = confirm_btn_skills;
            break;
        case 3:
            clicked_btn = confirm_btn_test;
            break;
        }

        if (!clicked_btn)
            return;

        // Check if clicking the currently highlighted button
        if (clicked_btn == currently_highlighted_btn)
        {
            // Unhighlight it
            lv_obj_set_style_bg_color(clicked_btn, lv_color_hex(CONFIRM_COLOR_NORMAL), 0);
            currently_highlighted_btn = nullptr;
            abclib::auton::selected_auton = abclib::auton::AutonRoutine::NONE;
        }
        else
        {
            // Unhighlight the old button (if any)
            if (currently_highlighted_btn != nullptr)
            {
                lv_obj_set_style_bg_color(currently_highlighted_btn, lv_color_hex(CONFIRM_COLOR_NORMAL), 0);
            }

            // Highlight the new button
            lv_obj_set_style_bg_color(clicked_btn, lv_color_hex(CONFIRM_COLOR_HIGHLIGHTED), 0);
            currently_highlighted_btn = clicked_btn;
            abclib::auton::selected_auton = temp_selection;
        }
    }

    void ScreenManager::populate_dropdown(lv_obj_t *dropdown, abclib::auton::AutonCategory category)
    {
        // Get all autons for this category in order
        std::vector<abclib::auton::AutonRoutine> autons = abclib::auton::get_autons_for_category(category);

        if (autons.empty())
        {
            lv_dropdown_set_options(dropdown, "NONE");
            return;
        }

        // Build options string (newline separated)
        std::string options = "";
        for (size_t i = 0; i < autons.size(); i++)
        {
            options += abclib::auton::get_display_name(autons[i]);
            if (i < autons.size() - 1)
            {
                options += "\n";
            }
        }

        lv_dropdown_set_options(dropdown, options.c_str());
    }

    void ScreenManager::setup_dropdown_callback(lv_obj_t *dropdown, abclib::auton::AutonCategory category)
    {
        lv_obj_add_event_cb(dropdown, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        lv_obj_t* dropdown = (lv_obj_t*)lv_event_get_target(e);
        
        // Get selected index
        uint32_t selected = lv_dropdown_get_selected(dropdown);
        
        // Get category from the dropdown's user data (we'll set this)
        abclib::auton::AutonCategory* category_ptr = (abclib::auton::AutonCategory*)lv_obj_get_user_data(dropdown);
        
        if (category_ptr) {
            // Get the list of autons for this category
            std::vector<abclib::auton::AutonRoutine> autons = 
                abclib::auton::get_autons_for_category(*category_ptr);
            
            // Set temp_selection to the selected auton
            if (selected < autons.size()) {
                manager->temp_selection = autons[selected];
            }
        } }, LV_EVENT_VALUE_CHANGED, this);
    }

    void ScreenManager::create_auton_telemetry_tab()
    {
        lv_obj_t *label = lv_label_create(auton_tab_telemetry);
        lv_label_set_text(label, "Return to Telemetry");
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    }

    void ScreenManager::create_auton_image_tab()
    {
        lv_obj_t *label = lv_label_create(auton_tab_image);
        lv_label_set_text(label, "Show Image");
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    }

    void ScreenManager::show_telemetry_screen()
    {
        // Hide autonomous screen
        lv_obj_add_flag(auton_tabview, LV_OBJ_FLAG_HIDDEN);

        // Show telemetry tabview and nav bar
        lv_obj_remove_flag(tabview, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(nav_bar, LV_OBJ_FLAG_HIDDEN);

        is_auton_screen_active = false;
    }

    void ScreenManager::show_autonomous_screen()
    {
        // Hide telemetry tabview and nav bar
        lv_obj_add_flag(tabview, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(nav_bar, LV_OBJ_FLAG_HIDDEN);

        // Show autonomous screen
        lv_obj_remove_flag(auton_tabview, LV_OBJ_FLAG_HIDDEN);

        // Set to first tab (Red)
        lv_tabview_set_active(auton_tabview, 0, LV_ANIM_OFF);

        is_auton_screen_active = true;
    }

    void ScreenManager::navigate_prev()
    {
        if (is_navigating || current_screen_index <= 0)
        {
            return; // Exit early if already navigating or at first screen
        }

        is_navigating = true;

        current_screen_index--;
        lv_tabview_set_active(tabview, current_screen_index, LV_ANIM_ON);
        update_current_screen_label();
        update_navigation_buttons();

        is_navigating = false;
    }

    void ScreenManager::navigate_next()
    {
        if (is_navigating || current_screen_index >= 5)
        {
            return; // Exit early if already navigating or at last screen
        }

        is_navigating = true;

        current_screen_index++;
        lv_tabview_set_active(tabview, current_screen_index, LV_ANIM_ON);
        update_current_screen_label();
        update_navigation_buttons();

        is_navigating = false;
    }

    void ScreenManager::update_current_screen_label()
    {
        const char *screen_names[] = {"Overview", "PID", "Trajectory", "Performance", "EKF", "Config"}; // CHANGED
        lv_label_set_text(label_current_screen, screen_names[current_screen_index]);
    }

    void ScreenManager::update_navigation_buttons()
    {
        // Hide/show left arrow
        if (current_screen_index == 0)
        {
            lv_obj_add_flag(btn_prev, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_remove_flag(btn_prev, LV_OBJ_FLAG_HIDDEN);
        }

        // Hide/show right arrow
        if (current_screen_index == 5)
        { // Last screen (index 4)
            lv_obj_add_flag(btn_next, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_remove_flag(btn_next, LV_OBJ_FLAG_HIDDEN);
        }
    }

    void ScreenManager::create_overview_tab()
    {
        // Create labels for overview data
        for (int i = 0; i < 4; i++)
        {
            lv_obj_t *label = lv_label_create(tab_overview);
            lv_label_set_text(label, "Loading...");
            lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 60 + i * 40);
            lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(label, 450);
            overview_labels.push_back(label);
        }
    }

    void ScreenManager::create_pid_tab()
    {
        lv_obj_t *label = lv_label_create(tab_pid);
        lv_label_set_text(label, "PID Control Data\n(Not implemented yet)");
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    }

    void ScreenManager::create_trajectory_tab()
    {
        lv_obj_t *label = lv_label_create(tab_trajectory);
        lv_label_set_text(label, "Trajectory Tracking Data\n(Not implemented yet)");
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    }

    void ScreenManager::create_performance_tab()
    {
        lv_obj_t *label = lv_label_create(tab_performance);
        lv_label_set_text(label, "Performance Metrics\n(Not implemented yet)");
        lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    }

    void ScreenManager::create_config_tab()
    {
        lv_obj_t *title = lv_label_create(tab_config);
        lv_label_set_text(title, "PID Tuner");
        lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
        lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

        for (int i = 0; i < 6; i++)
        {
            lv_obj_t *label = lv_label_create(tab_config);
            lv_label_set_text(label, "---");
            lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 50 + i * 30);
            lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
            lv_obj_set_width(label, 450);
            config_labels_.push_back(label);
        }
    }

    void ScreenManager::create_fullscreen_image()
    {
        // Create the full-screen image container (initially hidden)
        image_obj = lv_obj_create(lv_screen_active());
        lv_obj_set_size(image_obj, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_color(image_obj, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(image_obj, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(image_obj, 0, 0);
        lv_obj_set_style_pad_all(image_obj, 0, 0);

        // Check if the image file exists
        FILE *test_file = fopen("/usd/images/deft.bin", "r");
        bool image_exists = (test_file != nullptr);
        if (test_file)
        {
            fclose(test_file);
        }

        if (image_exists)
        {
            // Image exists - create and display it
            lv_obj_t *img = lv_image_create(image_obj);
            lv_image_set_src(img, "S:/images/deft.bin");
            lv_obj_align(img, LV_ALIGN_CENTER, 0, 0);
        }
        else
        {
            // Image doesn't exist - show error message
            lv_obj_t *error_label = lv_label_create(image_obj);
            lv_label_set_text(error_label,
                              "ERROR:\n"
                              "SD Card not found\n"
                              "or\n"
                              "Image file missing\n\n"
                              "Expected:\n"
                              "S:/images/deft.bin");
            lv_obj_set_style_text_color(error_label, lv_color_white(), 0);
            lv_obj_set_style_text_align(error_label, LV_TEXT_ALIGN_CENTER, 0);
            lv_obj_align(error_label, LV_ALIGN_CENTER, 0, 0);
        }

        // Make it clickable to close
        lv_obj_add_flag(image_obj, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(image_obj, [](lv_event_t *e)
                            {
        ScreenManager* manager = (ScreenManager*)lv_event_get_user_data(e);
        manager->hide_fullscreen_image(); }, LV_EVENT_CLICKED, this);

        // Hide it initially
        lv_obj_add_flag(image_obj, LV_OBJ_FLAG_HIDDEN);
    }

    void ScreenManager::show_fullscreen_image()
    {
        lv_obj_remove_flag(image_obj, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_to_index(image_obj, -1); // Move to front
    }

    void ScreenManager::hide_fullscreen_image()
    {
        lv_obj_add_flag(image_obj, LV_OBJ_FLAG_HIDDEN);

        // If we were on the autonomous screen, switch to telemetry tab
        if (is_auton_screen_active)
        {
            lv_tabview_set_active(auton_tabview, 4, LV_ANIM_OFF); // Tab 4 is Telemetry
        }
    }

    void ScreenManager::update_telemetry(const telemetry::TelemetryData &data)
    {
        update_overview_tab(data);
        update_blended_debug_tab(data);
        update_config_tab(data);
    }

    void ScreenManager::update_overview_tab(const telemetry::TelemetryData &data)
    {
        if (overview_labels.size() >= 4)
        {
            char buf0[128];
            char buf1[320];
            char buf2[128]; // Increased for uncertainty display
            char buf3[64];

            // Label 0: Alliance and Wall info
            const char *alliance_str = (data.current_alliance == field::Alliance::RED) ? "RED" : "BLUE";
            const char *wall_str = wall_to_string(data.heading_wall);
            snprintf(buf0, sizeof(buf0), "Alliance: %s | Wall: %s (%.1f\") [%s]",
                     alliance_str,
                     wall_str,
                     data.heading_distance_to_wall.to_inches(),
                     data.heading_wall_valid ? "OK" : "--");
            lv_label_set_text(overview_labels[0], buf0);

            // Label 1: Both poses - Corner frame and Math/Center frame
            snprintf(buf1, sizeof(buf1),
                     "Corner: X:%.1f Y:%.1f Th:%.1f | Center: X:%.1f Y:%.1f Th:%.1f",
                     data.pose_corner.x_inches(),
                     data.pose_corner.y_inches(),
                     data.pose_corner.theta_deg(),
                     data.pose_standard.x_inches(),
                     data.pose_standard.y_inches(),
                     data.pose_standard.theta_deg());
            lv_label_set_text(overview_labels[1], buf1);

            // Label 2: Velocity and Uncertainty
            if (data.has_covariance)
            {
                snprintf(buf2, sizeof(buf2),
                         "V:%.2f W:%.2f | Unc: Pos:%.2f\" Theta:%.1f",
                         data.pose_v_raw.to_ips(),
                         data.pose_omega_raw.to_rad_per_sec(),
                         data.position_uncertainty.to_inches(),
                         data.heading_uncertainty.to_degrees());
            }
            else
            {
                snprintf(buf2, sizeof(buf2), "V:%.2f W:%.2f | Unc: N/A",
                         data.pose_v_raw.to_ips(),
                         data.pose_omega_raw.to_rad_per_sec());
            }
            lv_label_set_text(overview_labels[2], buf2);

            // Label 3: Battery
            snprintf(buf3, sizeof(buf3), "%.2fV %.0f%% [%s%.2fx]",
                     data.battery_voltage.to_volts(),
                     data.battery_capacity_percent,
                     data.voltage_compensation_active ? "C" : "-",
                     data.voltage_compensation_scale);
            lv_label_set_text(overview_labels[3], buf3);
        }
    }

    void ScreenManager::create_calibration_screen()
    {
        // Create full-screen container
        calibration_screen = lv_obj_create(lv_screen_active());
        lv_obj_set_size(calibration_screen, LV_PCT(100), LV_PCT(100));
        lv_obj_set_style_bg_color(calibration_screen, lv_color_hex(0x1a1a1a), 0);
        lv_obj_remove_flag(calibration_screen, LV_OBJ_FLAG_SCROLLABLE);

        // Status label
        calibration_label = lv_label_create(calibration_screen);
        lv_label_set_text(calibration_label, "Calibrating IMU...");
        lv_obj_set_style_text_font(calibration_label, &lv_font_montserrat_24, 0);
        lv_obj_align(calibration_label, LV_ALIGN_CENTER, 0, -40);

        // Progress bar
        calibration_bar = lv_bar_create(calibration_screen);
        lv_obj_set_size(calibration_bar, 300, 30);
        lv_obj_align(calibration_bar, LV_ALIGN_CENTER, 0, 20);
        lv_bar_set_value(calibration_bar, 0, LV_ANIM_OFF);

        // Hide initially
        lv_obj_add_flag(calibration_screen, LV_OBJ_FLAG_HIDDEN);
    }

    void ScreenManager::show_calibration_screen()
    {
        lv_obj_remove_flag(calibration_screen, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_to_index(calibration_screen, -1); // Move to front
        lv_bar_set_value(calibration_bar, 0, LV_ANIM_OFF);
    }

    void ScreenManager::update_calibration_progress(int percentage, const char *status)
    {
        lv_bar_set_value(calibration_bar, percentage, LV_ANIM_ON);
        lv_label_set_text(calibration_label, status);
    }

    void ScreenManager::hide_calibration_screen()
    {
        lv_obj_add_flag(calibration_screen, LV_OBJ_FLAG_HIDDEN);
    }

    const char *ScreenManager::wall_to_string(field::FieldMap::Wall wall)
    {
        switch (wall)
        {
        case field::FieldMap::Wall::NORTH:
            return "N";
        case field::FieldMap::Wall::SOUTH:
            return "S";
        case field::FieldMap::Wall::EAST:
            return "E";
        case field::FieldMap::Wall::WEST:
            return "W";
        case field::FieldMap::Wall::NONE:
            return "-";
        default:
            return "?";
        }
    }

    // Stub implementations for tabs not yet implemented
    void ScreenManager::update_pid_tab(const telemetry::TelemetryData &data) {}
    void ScreenManager::update_trajectory_tab(const telemetry::TelemetryData &data) {}
    void ScreenManager::update_performance_tab(const telemetry::TelemetryData &data) {}
    void ScreenManager::update_config_tab(const telemetry::TelemetryData &data)
    {
        if (!pid_tuner_ || config_labels_.size() < 6)
            return;

        const auto &gains = pid_tuner_->get_gains();
        int cursor = pid_tuner_->get_cursor();
        int move_count = pid_tuner_->get_move_count();
        sysid::TunerState state = pid_tuner_->get_state();

        for (int i = 0; i < static_cast<int>(gains.size()) && i < 6; i++)
        {
            char buf[64];
            if (i == cursor)
            {
                snprintf(buf, sizeof(buf), "> %s: %.4f  [+/-: %.3f] moves:%d",
                         gains[i].name,
                         *gains[i].value,
                         gains[i].increment,
                         move_count);
                lv_obj_set_style_text_color(config_labels_[i], lv_color_hex(0xFFFF00), 0);
            }
            else
            {
                snprintf(buf, sizeof(buf), "  %s: %.4f",
                         gains[i].name,
                         *gains[i].value);
                lv_obj_set_style_text_color(config_labels_[i], lv_color_white(), 0);
            }
            lv_label_set_text(config_labels_[i], buf);
        }
    }

} // namespace abclib