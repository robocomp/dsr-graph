//
// Created by pbustos on 5/4/21.
//

#ifndef CONTROLLER_DSR_PLAN_H
#define CONTROLLER_DSR_PLAN_H

class Plan
{
    public:
        enum class Actions {GOTO};
        Actions action;
        std::string target_place;
        std::map<std::string, double> params;
        void set_active(bool s) {active = s;};
        bool is_active() const {return active;};
        bool is_location(const Mat::Vector2d &loc)
        {
            return Mat::Vector2d(params.at("x"), params.at("y")) == loc;
        }
        void print()
        {
            std::cout << "------ Begin Plan ----------" << std::endl;
            std::cout << "\t Action: " << action_strings[action] << " Target_place: " << target_place << std::endl;
            for(auto &&[k,v]: params)
                std::cout << "\t par1: " << k << " : " << std::to_string(v) << std::endl;
            std::cout << "------ End Plan ----------" << std::endl;
        };
        std::string to_string() const { return plan_string; }
        std::string pprint()
        {
            std::stringstream ss;
            ss <<  std::setprecision(2) << std::fixed ;
            ss << "Intention: " << action_strings[action] << std::endl;
            ss << "      location: " << target_place << std::endl;
            ss << "      final pose: " << target_place << std::endl;
            ss.precision(2);
            for(auto &&[k,v]: params)
                ss << std::fixed << "\t" << k << " : " << std::to_string((float)v) << std::endl;  //OJO PONER EN FLOAT
           return ss.str();
        };
        //////////////////////////////////////////////
        /// parser form JSON plan to Plan structure
        //////////////////////////////////////////////
        Plan() = default;
        Plan(const std::string &plan_string_)
        {
            std::cout << __FUNCTION__ << plan_string_ << std::endl;
            QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(plan_string_).toUtf8());
            QJsonObject planJson = doc.object();
            QJsonArray actionArray = planJson.value("plan").toArray();
            QJsonObject action_0 = actionArray.at(0).toObject();
            QString action_s = action_0.value("action").toString();
            qInfo() << __FUNCTION__ << action_0 << action_s;
            if (action_s == "goto")
            {
                QJsonObject action_params = action_0.value("params").toObject();
                QString object = action_params.value("object").toString();
                QJsonArray location = action_params.value("location").toArray();
                params["x"] = location.at(0).toDouble();
                params["y"] = location.at(1).toDouble();
                params["angle"] = location.at(2).toDouble();
                action = Plan::Actions::GOTO;
                target_place = object.toStdString();
            }
            plan_string = plan_string_;
        };

        std::string plan_string;
    private:
        std::map<Actions, std::string> action_strings{{Actions::GOTO, "GOTO"}};
        bool active = false;
};


#endif //CONTROLLER_DSR_MISSION_H
