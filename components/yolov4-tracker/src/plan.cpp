
#include "plan.h"

//////////////////////////////////////////////
/// parser form JSON plan to Plan structure
//////////////////////////////////////////////7

void Plan::json_to_plan(const std::string &plan_string)
{
    QJsonDocument doc = QJsonDocument::fromJson(QString::fromStdString(plan_string).toUtf8());
    QJsonObject planJson = doc.object();
    QJsonArray actionArray = planJson.value("plan").toArray();
    QJsonObject action_0 = actionArray.at(0).toObject();
    QString action = action_0.value("action").toString();
    if (action == "goto")
    {
        QJsonObject action_params = action_0.value("params").toObject();
        QString object = action_params.value("object").toString();
        QJsonArray location = action_params.value("location").toArray();
        this->params["x"] = location.at(0).toDouble();
        this->params["y"] = location.at(1).toDouble();
        this->params["angle"] = location.at(2).toDouble();
        this->action = Actions::GOTO;
        this->target_place = object.toStdString();
    }
}


