#include "console.hpp"
#include <ncurses.h>
#include <iterator>
#include <sstream>
#include <unistd.h>
#include <functional>
#include <locale.h>

using namespace std;

mutex cons_mtx_;
int refresh_flag;

void Console::init(FpgaHandler* fpga,
                   LegModule* legs_,
                   LegModule* servos_,
                   std::vector<bool>* pb_state_ptr_,
                   ModeFsm* fsm_ptr_,
                   std::mutex* mtx_ptr_){
    fpga_ = fpga;
    this->legs_ = legs_;
    this->servos_ = servos_;

    setlocale(LC_ALL, "");
    initscr();
    getmaxyx(stdscr, term_max_y_, term_max_x_);
    start_color();
    init_pair(BKGD_PAIR, COLOR_WHITE, COLOR_BLACK);
    wbkgd(stdscr, COLOR_PAIR(BKGD_PAIR));
    init_pair(CYAN_PAIR, COLOR_CYAN, COLOR_BLACK);

    frontend_rate_ = 3;
    input_panel_.main_mtx_ = mtx_ptr_;
    input_panel_.powerboard_state_ = pb_state_ptr_;
    input_panel_.fsm_ = fsm_ptr_;
    input_panel_.legs_ = legs_;
    input_panel_.servos_ = servos_;

    main_mtx_ = mtx_ptr_;
    powerboard_state_ = pb_state_ptr_;
    fsm_ = fsm_ptr_;

    if_resetPanel = false;
    t_frontend_ = thread(&Console::refreshWindow, this);
    refresh_flag = 1;
    input_panel_.init(legs_, servos_, &if_resetPanel, term_max_x_, term_max_y_);

}

void Console::refreshWindow()
{
    clear();
  
    LegModule *leg_null = 0;
    const int reserved_bottom = 3;    
    const int top_margin = 1;
    const int gap = 1;

    const int left_x = 1;
    const int left_w = 40;

    const int top_h = 8;
    const int avail_h = term_max_y_ - reserved_bottom - top_margin;
    const int power_y = top_margin + top_h + gap;
    const int power_h = std::max(3, avail_h - top_h - gap);

    const int right_x = left_x + left_w + gap;
    const int right_w = std::max(20, term_max_x_ - right_x - 1);
    const int right_y = top_margin;
    const int right_h = std::max(5, avail_h);


    Panel p_cmain_ ("[F] FPGA Server ", "c_main", leg_null, nullptr, left_x, top_margin, top_h, left_w, true);
    Panel p_power_ ("[P] Power Board ", "power",   leg_null, nullptr, left_x, power_y, power_h, left_w, true);
    Panel p_legs_  ("[L] Legs ", "legs", this->legs_, nullptr, right_x, right_y, right_h, right_w, true);


    p_power_.powerboard_state_ = powerboard_state_;
    p_cmain_.fsm_ = fsm_;

    while(1){
        cons_mtx_.lock();
        main_mtx_->lock();  
        p_power_.infoDisplay(fpga_, powerboard_state_->at(0), powerboard_state_->at(1), powerboard_state_->at(2));
        p_cmain_.infoDisplay(Behavior::TCP_SLAVE, fsm_->workingMode_);
        p_legs_.infoDisplay();
        cons_mtx_.unlock();
        main_mtx_->unlock();
        usleep(0.1 * 1000 * 1000);
    }
}


void InputPanel::init(LegModule *legs_, LegModule *servos_, bool *if_resetPanel, int term_max_x, int term_max_y){
    win_ = newwin(3, term_max_x - 1, term_max_y - 3, 1);
    //this->if_resetPanel = if_resetPanel_;

    thread_ = new std::thread(&InputPanel::inputHandler, this, win_, std::ref(mutex_));
}

void InputPanel::inputHandler(WINDOW *win_, std::mutex &input_mutex){
    while(1){
        int ch = 0;
        int x = 1;
        do{
            ch = mvwgetch(win_, 1, x);
            if (ch == 'r'){
                reset_input_window(win_);
            }
            if (ch == 'e'){
                endwin();
                std::cout << "Normal Mode" << std::endl;
                refresh_flag = 0;
            }
            if (ch == 'E'){
                refresh_flag = 1;
                refresh();
            }

        }while (ch != ':');

        string input_buf;
        keypad(win_, true);
        do{
            ch = mvwgetch(win_, 1, x);

            if (ch == KEY_BACKSPACE || ch == KEY_DC || ch == 127){
                mvwdelch(win_, 1, x);
                mvwdelch(win_, 1, x + 1);
                mvwdelch(win_, 1, x - 1);
                wclrtoeol(win_);
                wrefresh(win_);

                if (input_buf.size() > 0){
                    input_buf.erase(input_buf.size() - 1, 1);
                    x--;
                }
            }
            else
            {
                if (ch != '\n'){
                    input_buf.append(1, ch);
                }
                x++;
            }
        }while (ch != '\n');

        reset_input_window(win_);
        commandDecode(input_buf);
    }
}


void InputPanel::reset_input_window(WINDOW *win)
{
    werase(win);
    wclear(win);
    wrefresh(win);
}

void InputPanel::commandDecode(const std::string& buf)
{
    bool syntax_err = false;
    bool pb_selected = false;
    bool legs_selected = false;
    bool f_selected = false;
    bool switchFSM_success = true;

    vector<string> bufs;
    bufs = tokenizer(buf);

    if (bufs.size() >= 1){
        if (bufs[0] == "P"){
            pb_selected = true;
        }
        else if (bufs[0] == "F"){
            f_selected = true;
        }
        else if (bufs[0] == "L"){
            legs_selected = true;
        }
        else{
            syntax_err = true;
        }

        mvwprintw(win_, 2, 1, "%s", bufs[0].c_str());
        wrefresh(win_);
    }

    cons_mtx_.lock();
    main_mtx_->lock();

    if (bufs.size() == 3){
        mvwprintw(win_, 2, 3, "%s", bufs[1].c_str());
        mvwprintw(win_, 2, 5, "%s", bufs[2].c_str());
        wrefresh(win_);

        if (pb_selected){
            if (bufs[1] == "D") {
                try{
                    powerboard_state_->at(0) = stoi(bufs[2]);
                }
                catch (exception &e){
                    syntax_err = true;
                    mvwprintw(win_, 2, 1, "err");
                }
            }
            else if (bufs[1] == "S"){
                try{
                    powerboard_state_->at(1) = stoi(bufs[2]);
                }
                catch (exception &e){
                    syntax_err = true;
                    mvwprintw(win_, 2, 1, "err");
                }
            }
            else if (bufs[1] == "P"){
                try{
                    powerboard_state_->at(2) = stoi(bufs[2]);
                }
                catch (exception &e){
                    syntax_err = true;
                    mvwprintw(win_, 2, 1, "err");
                }
            }
            else{
                syntax_err = true;
            }
        }
        else if (f_selected){
            if (bufs[1] == "M"){
                if (bufs[2] == "R"){
                    switchFSM_success = fsm_->switchMode(Mode::REST);
                }
                else if (bufs[2] == "M"){
                    switchFSM_success = fsm_->switchMode(Mode::MOTOR);
                }
                else if (bufs[2] == "S"){
                    switchFSM_success = fsm_->switchMode(Mode::SET_ZERO);
                }
                else if (bufs[2] == "H"){
                    switchFSM_success = fsm_->switchMode(Mode::HALL_CALIBRATE);
                }
                else{
                    syntax_err = true;
                }
            }
            else{
                syntax_err = true;
            }
        }
        else{
            syntax_err = true;
        }
    }
    else if (bufs.size() >= 4 && legs_selected){
        mvwprintw(win_, 2, 3, "%s", bufs[1].c_str());
        mvwprintw(win_, 2, 5, "%s", bufs[2].c_str());
        wrefresh(win_);

        int idx = 0;
        
        if (bufs[2] == "A"){
            try{
                legs_->txdata_buffer_[idx].position_ = std::stof(bufs[3]);
            }
            catch (exception &e){
                syntax_err = true;
            }
        }
        else if (bufs[2] == "T"){
            try{
                legs_->txdata_buffer_[idx].torque_ = std::stof(bufs[3]);
            }
            catch (exception &e){
                syntax_err = true;
            }
        }
        else if (bufs[2] == "P"){
            try{
                legs_->txdata_buffer_[idx].KP_ = std::stof(bufs[3]);
            }
            catch (exception &e){
                syntax_err = true;
            }
        }
        else if (bufs[2] == "I"){
            try{
                legs_->txdata_buffer_[idx].KI_ = std::stof(bufs[3]);
            }
            catch (exception &e){
                syntax_err = true;
            }
        }
        else if (bufs[2] == "D"){
            try{
                legs_->txdata_buffer_[idx].KD_ = std::stof(bufs[3]);
            }
            catch (const std::exception&){
                syntax_err = true;
            }
        }
        else{
            syntax_err = true;
        }
        
    }
    else{
        syntax_err = true;
    }

    if (syntax_err){
        mvwprintw(win_, 0, 1, "Syntax Error !");
    }
    else if (!switchFSM_success){
        mvwprintw(win_, 0, 1, "Switch Mode Timeout !");
    }
    else{
        mvwprintw(win_, 0, 1, "Command Send !");
    }

    cons_mtx_.unlock();
    main_mtx_->unlock();

    wrefresh(win_);
}

std::vector<std::string> InputPanel::tokenizer(const std::string& s){
    // A quick way to split strings separated via spaces.
    stringstream ss(s);
    string word;
    vector<string> bufs;
    while (ss >> word){
        // cout << word << endl;
        bufs.push_back(word);
    }
    return bufs;
}


Panel::Panel(const std::string& title,
             const std::string& type,
             LegModule* legs_,
             LegModule* servos_,
             int org_x,
             int org_y,
             int height,
             int width,
             bool box_on){
    org_x_ = org_x;
    org_y_ = org_y;
    height_ = height;
    width_ = width;
    type_ = type;
    title_ = title;
    this->legs_ = legs_;
    this->servos_ = servos_;

    win_ = newwin(height_, width_, org_y_, org_x_);
    refresh();

    string tag_(title.c_str(), title.c_str() + 3);
    if (title_.size() >= 3) title_.erase(0, 3);

    // mvwprintw(win_, 0, (width_ / 2 - (title.size() + tag_.size()) / 2), tag_.c_str());
    wattron(win_, COLOR_PAIR(CYAN_PAIR));
    wattron(win_, A_BOLD);
    wattron(win_, A_STANDOUT);
    mvwprintw(win_, 0, (width_ / 2 - title.size() / 2 - 2), tag_.c_str());
    wattroff(win_, COLOR_PAIR(CYAN_PAIR));

    mvwprintw(win_, 0, (width_ / 2 - title.size() / 2 + 1), title.c_str());
    wattroff(win_, A_BOLD);
    wattroff(win_, A_STANDOUT);
    wrefresh(win_);
        // refresh();
}

void Panel::infoDisplay(){

    int y_org = 2;

    mvwprintw(win_, 1, 1, "[L] L1 -----------------------------------------");
    mvwprintw(win_, 2, 1, "[E] [EN] enable: %9d", legs_->motors_list_[0].enable_);
    mvwprintw(win_, y_org + 1, 1,  "[A] [tx] Pos:  %5.5f", legs_->txdata_buffer_[0].position_);
    mvwprintw(win_, y_org + 2, 1,  "[T] [tx] Trq:  %5.5f", legs_->txdata_buffer_[0].torque_);
    mvwprintw(win_, y_org + 3, 1,  "[P] [tx] KP:   %4.5f", legs_->txdata_buffer_[0].KP_);
    mvwprintw(win_, y_org + 4, 1,  "[I] [tx] KI:   %5.5f", legs_->txdata_buffer_[0].KI_);
    mvwprintw(win_, y_org + 5, 1,  "[D] [tx] KD:   %5.5f", legs_->txdata_buffer_[0].KD_);
    // reply
    mvwprintw(win_, y_org + 1, 30, "[rx] Mode:  %7d", 0);
    mvwprintw(win_, y_org + 2, 30, "[rx] Pos:   %4.5f", legs_->rxdata_buffer_[0].position_);
    mvwprintw(win_, y_org + 3, 30, "[rx] Vel:   %4.5f", legs_->rxdata_buffer_[0].velocity_);
    mvwprintw(win_, y_org + 4, 30, "[rx] Trq:   %4.5f", legs_->rxdata_buffer_[0].torque_);
    mvwprintw(win_, y_org + 5, 30, "[rx] Cal:   %7d", legs_->rxdata_buffer_[0].calibrate_finish_);
    wrefresh(win_);

    mvwprintw(win_, 10, 1, "[L] L2 -----------------------------------------");
    mvwprintw(win_, 11, 1, "[E] [EN] enable: %9d", legs_->motors_list_[1].enable_);
    mvwprintw(win_, y_org + 10, 1,  "[A] [tx] Pos:  %5.5f", legs_->txdata_buffer_[1].position_);
    mvwprintw(win_, y_org + 11, 1,  "[T] [tx] Trq:  %5.5f", legs_->txdata_buffer_[1].torque_);
    mvwprintw(win_, y_org + 12, 1,  "[P] [tx] KP:   %4.5f", legs_->txdata_buffer_[1].KP_);
    mvwprintw(win_, y_org + 13, 1,  "[I] [tx] KI:   %5.5f", legs_->txdata_buffer_[1].KI_);
    mvwprintw(win_, y_org + 14, 1,  "[D] [tx] KD:   %5.5f", legs_->txdata_buffer_[1].KD_);
    // reply
    mvwprintw(win_, y_org + 10, 30, "[rx] Mode:  %7d", 0);
    mvwprintw(win_, y_org + 11, 30, "[rx] Pos:   %4.5f", legs_->rxdata_buffer_[1].position_);
    mvwprintw(win_, y_org + 12, 30, "[rx] Vel:   %4.5f", legs_->rxdata_buffer_[1].velocity_);
    mvwprintw(win_, y_org + 13, 30, "[rx] Trq:   %4.5f", legs_->rxdata_buffer_[1].torque_);
    mvwprintw(win_, y_org + 14, 30, "[rx] Cal:   %7d", legs_->rxdata_buffer_[1].calibrate_finish_);
    wrefresh(win_);

    mvwprintw(win_, 20, 1, "[L] L3 -----------------------------------------");
    mvwprintw(win_, 21, 1, "[E] [EN] enable: %9d", legs_->motors_list_[2].enable_);
    mvwprintw(win_, y_org + 20, 1,  "[A] [tx] Pos:  %5.5f", legs_->txdata_buffer_[2].position_);
    mvwprintw(win_, y_org + 21, 1,  "[T] [tx] Trq:  %5.5f", legs_->txdata_buffer_[2].torque_);
    mvwprintw(win_, y_org + 22, 1,  "[P] [tx] KP:   %4.5f", legs_->txdata_buffer_[2].KP_);
    mvwprintw(win_, y_org + 23, 1,  "[I] [tx] KI:   %5.5f", legs_->txdata_buffer_[2].KI_);
    mvwprintw(win_, y_org + 24, 1,  "[D] [tx] KD:   %5.5f", legs_->txdata_buffer_[2].KD_);
    // reply
    mvwprintw(win_, y_org + 20, 30, "[rx] Mode:  %7d", 0);
    mvwprintw(win_, y_org + 21, 30, "[rx] Pos:   %4.5f", legs_->rxdata_buffer_[2].position_);
    mvwprintw(win_, y_org + 22, 30, "[rx] Vel:   %4.5f", legs_->rxdata_buffer_[2].velocity_);
    mvwprintw(win_, y_org + 23, 30, "[rx] Trq:   %4.5f", legs_->rxdata_buffer_[2].torque_);
    mvwprintw(win_, y_org + 24, 30, "[rx] Cal:   %7d", legs_->rxdata_buffer_[2].calibrate_finish_);
    wrefresh(win_);

    mvwprintw(win_, 1, 60, "[L] R1 -----------------------------------------");
    mvwprintw(win_, 2, 60, "[E] [EN] enable: %9d", legs_->motors_list_[3].enable_);
    mvwprintw(win_, y_org + 1, 60,  "[A] [tx] Pos:  %5.5f", legs_->txdata_buffer_[3].position_);
    mvwprintw(win_, y_org + 2, 60,  "[T] [tx] Trq:  %5.5f", legs_->txdata_buffer_[3].torque_);
    mvwprintw(win_, y_org + 3, 60,  "[P] [tx] KP:   %4.5f", legs_->txdata_buffer_[3].KP_);
    mvwprintw(win_, y_org + 4, 60,  "[I] [tx] KI:   %5.5f", legs_->txdata_buffer_[3].KI_);
    mvwprintw(win_, y_org + 5, 60,  "[D] [tx] KD:   %5.5f", legs_->txdata_buffer_[3].KD_);
    // reply
    mvwprintw(win_, y_org + 1, 90, "[rx] Mode:  %7d", 0);
    mvwprintw(win_, y_org + 2, 90, "[rx] Pos:   %4.5f", legs_->rxdata_buffer_[3].position_);
    mvwprintw(win_, y_org + 3, 90, "[rx] Vel:   %4.5f", legs_->rxdata_buffer_[3].velocity_);
    mvwprintw(win_, y_org + 4, 90, "[rx] Trq:   %4.5f", legs_->rxdata_buffer_[3].torque_);
    mvwprintw(win_, y_org + 5, 90, "[rx] Cal:   %7d", legs_->rxdata_buffer_[3].calibrate_finish_);
    wrefresh(win_);

    mvwprintw(win_, 10, 60, "[L] R2 -----------------------------------------");
    mvwprintw(win_, 11, 60, "[E] [EN] enable: %9d", legs_->motors_list_[4].enable_);
    mvwprintw(win_, y_org + 10, 60,  "[A] [tx] Pos:  %5.5f", legs_->txdata_buffer_[4].position_);
    mvwprintw(win_, y_org + 11, 60,  "[T] [tx] Trq:  %5.5f", legs_->txdata_buffer_[4].torque_);
    mvwprintw(win_, y_org + 12, 60,  "[P] [tx] KP:   %4.5f", legs_->txdata_buffer_[4].KP_);
    mvwprintw(win_, y_org + 13, 60,  "[I] [tx] KI:   %5.5f", legs_->txdata_buffer_[4].KI_);
    mvwprintw(win_, y_org + 14, 60,  "[D] [tx] KD:   %5.5f", legs_->txdata_buffer_[4].KD_);
    // reply
    mvwprintw(win_, y_org + 10, 90, "[rx] Mode:  %7d", 0);
    mvwprintw(win_, y_org + 11, 90, "[rx] Pos:   %4.5f", legs_->rxdata_buffer_[4].position_);
    mvwprintw(win_, y_org + 12, 90, "[rx] Vel:   %4.5f", legs_->rxdata_buffer_[4].velocity_);
    mvwprintw(win_, y_org + 13, 90, "[rx] Trq:   %4.5f", legs_->rxdata_buffer_[4].torque_);
    mvwprintw(win_, y_org + 14, 90, "[rx] Cal:   %7d", legs_->rxdata_buffer_[4].calibrate_finish_);
    wrefresh(win_);

    mvwprintw(win_, 20, 60, "[L] R3 -----------------------------------------");
    mvwprintw(win_, 21, 60, "[E] [EN] enable: %9d", legs_->motors_list_[5].enable_);
    mvwprintw(win_, y_org + 20, 60,  "[A] [tx] Pos:  %5.5f", legs_->txdata_buffer_[5].position_);
    mvwprintw(win_, y_org + 21, 60,  "[T] [tx] Trq:  %5.5f", legs_->txdata_buffer_[5].torque_);
    mvwprintw(win_, y_org + 22, 60,  "[P] [tx] KP:   %4.5f", legs_->txdata_buffer_[5].KP_);
    mvwprintw(win_, y_org + 23, 60,  "[I] [tx] KI:   %5.5f", legs_->txdata_buffer_[5].KI_);
    mvwprintw(win_, y_org + 24, 60,  "[D] [tx] KD:   %5.5f", legs_->txdata_buffer_[5].KD_);
    // reply
    mvwprintw(win_, y_org + 20, 90, "[rx] Mode:  %7d", 0);
    mvwprintw(win_, y_org + 21, 90, "[rx] Pos:   %4.5f", legs_->rxdata_buffer_[5].position_);
    mvwprintw(win_, y_org + 22, 90, "[rx] Vel:   %4.5f", legs_->rxdata_buffer_[5].velocity_);
    mvwprintw(win_, y_org + 23, 90, "[rx] Trq:   %4.5f", legs_->rxdata_buffer_[5].torque_);
    mvwprintw(win_, y_org + 24, 90, "[rx] Cal:   %7d", legs_->rxdata_buffer_[5].calibrate_finish_);
    wrefresh(win_);
}


void Panel::infoDisplay(FpgaHandler* fpga,
                        bool power_switch,
                        bool signal_switch,
                        bool digital_switch){
    mvwprintw(win_, 2, 1, "HARDWARE POWER SWITCH ----------------");
    mvwprintw(win_, 3, 1, "[D] Digital:   %4d", digital_switch);
    mvwprintw(win_, 4, 1, "[S] Signal:    %4d", signal_switch);
    mvwprintw(win_, 5, 1, "[P] Power:     %4d", power_switch);

    mvwprintw(win_, 6, 1, "Voltage Current ADC ------------------");
    mvwprintw(win_, 7, 1, "Voltage: %5.5f, Current: %5.5f", fpga->powerboard_V_list_[0], fpga->powerboard_I_list_[0]);
    mvwprintw(win_, 8, 1, "Voltage: %5.5f, Current: %5.5f", fpga->powerboard_V_list_[1], fpga->powerboard_I_list_[1]);
    mvwprintw(win_, 9, 1, "Voltage: %5.5f, Current: %5.5f", fpga->powerboard_V_list_[2], fpga->powerboard_I_list_[2]);
    mvwprintw(win_, 10, 1, "Voltage: %5.5f, Current: %5.5f", fpga->powerboard_V_list_[3], fpga->powerboard_I_list_[3]);
    mvwprintw(win_, 11, 1, "Voltage: %5.5f, Current: %5.5f", fpga->powerboard_V_list_[4], fpga->powerboard_I_list_[4]);
    mvwprintw(win_, 12, 1, "Voltage: %5.5f, Current: %5.5f", fpga->powerboard_V_list_[5], fpga->powerboard_I_list_[5]);
    mvwprintw(win_, 13, 1, "Voltage: %5.5f, Current: %5.5f", fpga->powerboard_V_list_[6], fpga->powerboard_I_list_[6]);

    wrefresh(win_);
}

void Panel::infoDisplay(Behavior bhv, Mode fsm_mode){
    if (bhv == Behavior::TCP_SLAVE)
        mvwprintw(win_, 2, 1, "Behavior: TCP_SLAVE");
    else if (bhv == Behavior::SET_THETA)
        mvwprintw(win_, 2, 1, "Behavior: SET_THETA");
    else if (bhv == Behavior::SET_SERVO)
        mvwprintw(win_, 2, 1, "Behavior: SET_SERVO");
    else if (bhv == Behavior::CUSTOM_1)
        mvwprintw(win_, 2, 1, "Behavior: CUSTOM_1");
    else if (bhv == Behavior::CUSTOM_2)
        mvwprintw(win_, 2, 1, "Behavior: CUSTOM_2");
    else if (bhv == Behavior::CUSTOM_3)
        mvwprintw(win_, 2, 1, "Behavior: CUSTOM_3");

    if (fsm_mode == Mode::REST)
        mvwprintw(win_, 3, 1, "[M] FSM Mode:  REST");
    else if (fsm_mode == Mode::SET_ZERO)
        mvwprintw(win_, 3, 1, "[M] FSM Mode: SET_ZERO");
    else if (fsm_mode == Mode::HALL_CALIBRATE)
        mvwprintw(win_, 3, 1, "[M] FSM Mode: HALL_CALIBRATE");
    else if (fsm_mode == Mode::MOTOR)
        mvwprintw(win_, 3, 1, "[M] FSM Mode: MOTOR");

    mvwprintw(win_, 5, 1, "[R] REST  [S] SET_ZERO ");
    mvwprintw(win_, 6, 1, "[M] MOTOR [H] HALL_CALIBRATE");

    wrefresh(win_);
}

void Panel::panelTitle(){
    string tag_(title_.c_str(), title_.c_str() + 3);
    title_.erase(0, 3);
    wattron(win_, COLOR_PAIR(CYAN_PAIR));
    wattron(win_, A_BOLD);
    wattron(win_, A_STANDOUT);
    mvwprintw(win_, 0, (width_ / 2 - title_.size() / 2 - 2), tag_.c_str());
    wattroff(win_, COLOR_PAIR(CYAN_PAIR));

    mvwprintw(win_, 0, (width_ / 2 - title_.size() / 2 + 1), title_.c_str());
    wattroff(win_, A_BOLD);
    wattroff(win_, A_STANDOUT);

    wrefresh(win_);
}

void Panel::resetPanel()
{
    werase(win_);
    wclear(win_);
    wrefresh(win_);
}

// ok
