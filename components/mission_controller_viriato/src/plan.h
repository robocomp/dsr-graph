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
    bool is_active = false;
    bool is_location(const Mat::Vector2d &loc)
    {
        return Mat::Vector2d(params.at("x"), params.at("y")) == loc;
    }
    void print()
    {
        std::cout << "------ Begin Plan ----------" << std::endl;
        std::cout << "\t Action: " << action_strings[action] << " Taget_place: " << target_place << std::endl;
        for(auto &&[k,v]: params)
            std::cout << "\t par1: " << k << " : " << std::to_string(v) << std::endl;
        std::cout << "------ End Plan ----------" << std::endl;
    };
    std::string to_string() const { return plan_string; }
    std::string pprint()
    {
        std::stringstream ss;
        ss << "Intention: " << action_strings[action] << std::endl;
        ss << "      location: " << target_place << std::endl;
        ss << "      final pose: " << target_place << std::endl;
        for(auto &&[k,v]: params)
            ss << "\t" << k << " : " << std::to_string(v) << std::endl;
       return ss.str();
    };
    //////////////////////////////////////////////7
    /// parser form JSON plan to Plan structure
    Plan(){};

    Plan(const std::string &plan_string_)
    {
        QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(plan_string_).toUtf8());
        QJsonObject planJson = doc.object();
        QJsonArray actionArray = planJson.value("plan").toArray();
        QJsonObject action_0 = actionArray.at(0).toObject();
        QString action_s = action_0.value("action").toString();
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
};


#endif //CONTROLLER_DSR_MISSION_H
