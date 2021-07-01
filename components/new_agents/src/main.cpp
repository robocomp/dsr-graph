/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */


/** \mainpage RoboComp::idserver
 *
 * \section intro_sec Introduction
 *
 * The idserver component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd idserver
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/idserver --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <csignal>
#include <cxxabi.h>
#include <filesystem>

// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
//#include <Ice/Ice.h>
//#include <IceStorm/IceStorm.h>
//#include <Ice/Application.h>

#include <sigwatch/sigwatch.h>

#include "config.h"
#include "genericworker.h"
#include "specificworker.h"


bool configGetString(const std::unordered_map<std::string, std::string>& properties,  const std::string& prefix, const std::string& name, std::string& value, const std::string& default_value, QStringList *list = nullptr)
{
    std::string compound = name;
    if (not prefix.empty()) compound = prefix+std::string(".")+name;

    value = "";
    if (auto it = properties.find(compound); it != properties.end())
    {
        value = it->second;
    }

    if (value.empty())
    {
        if (not default_value.empty())
        {
            value = default_value;
            return false;
        }
        else
        {
            QString error = QString("Error: can't get configuration string for variable without default value: ")+QString::fromStdString(compound);
            qDebug() << error;
            throw std::runtime_error(error.toStdString());
        }
    }

    if (list != nullptr)
    {
        if (!list->contains(QString::fromStdString(value)))
        {
            qFatal("Reading config file: %s is not a valid string", compound.c_str());
            //rError("Reading config file:"+compound+" is not a valid string");
        }
        QString error = QString("not valid configuration value");
        qDebug() << error;
        throw std::runtime_error(error.toStdString());
    }

    auto parts = QString::fromStdString(value).split("@");
    QString variableName=QString::fromStdString(compound);


    if (parts.size() > 1)
    {
        if (parts[0].size() > 0)
        {
            variableName = parts[0];
        }
        parts.removeFirst();
        value = std::string("@") + parts.join("@").toStdString();
    }


    if (value[0]=='@')
    {
        QString qstr = QString::fromStdString(value).remove(0,1);
        QFile ff(qstr);
        if (not ff.exists())
        {
            qFatal("Not such file: %s\n", qstr.toStdString().c_str());
        }
        if (!ff.open(QIODevice::ReadOnly | QIODevice::Text))
        {
            qFatal("Can't open file: %s\n", qstr.toStdString().c_str());
        }

        bool found = false;
        while (!ff.atEnd())
        {
            QString content = QString(ff.readLine()).simplified();
// 			printf("line: %s\n", content.toStdString().c_str());

            if (content.startsWith(variableName))
            {
// 				printf("swn %s\n", content.toStdString().c_str());
                content = content.right(content.size()-variableName.size()).simplified();
// 				printf("swn %s\n", content.toStdString().c_str());
                if (content.startsWith("="))
                {
                    content = content.remove(0,1).simplified();
                    value = content.toStdString();
                    found = true;
                }
                else
                {
                    printf("warning (=) %s\n", content.toStdString().c_str());
                }

            }
        }
        if (not found)
        {
        }
    }
    std::cout << compound << " " << value << std::endl;
    return true;
}

void setConfig(std::unordered_map<std::string, std::string> &params)
{
    std::string aux;
    configGetString(params, "","agent_name", aux,"");
    params["agent_name"] = aux;
    configGetString(params, "","agent_id", aux,"false");
    params["agent_id"] = aux;
    configGetString(params, "","dsr_input_file", aux, "");
    params["dsr_input_file"] = aux;
    configGetString(params, "","dsr_output_path", aux, "");
    params["dsr_output_path"] = aux;
    configGetString(params, "","period", aux, "10000");
    params["period"] = aux;
    configGetString(params, "","dsr_write_to_file", aux, "false");
    params["dsr_write_to_file"] = aux;
    configGetString(params, "","tree_view", aux, "none");
    params["tree_view"] = aux;
    configGetString(params, "","graph_view", aux, "none");
    params["graph_view"] = aux;
    configGetString(params, "","2d_view", aux, "none");
    params["2d_view"] = aux;
    configGetString(params, "","3d_view", aux, "none");
    params["3d_view"] = aux;
}

std::unordered_map<std::string, std::string> parseConfig(const std::string &file)
{

    std::unordered_map<std::string, std::string> config;
    std::filesystem::path path_file = std::filesystem::relative(file);

    const std::string_view spaces = " \t\n\r\f\v";
    const auto trim = [&](std::string & str) {
        str.erase(str.find_last_not_of(spaces) + 1);
        str.erase(0, str.find_first_not_of(spaces));
    };

    //Check that the file exists.
    if (std::filesystem::exists(path_file))
    {
        std::ifstream file_content(path_file.c_str());
        std::vector<std::string> lines;
        std::string line_content;

        //For each line in the file.
        while(std::getline(file_content, line_content))
        {
            //Skip empty lines and comments
            if (not line_content.empty() and not line_content.starts_with('#'))
            {
                lines.emplace_back(std::move(line_content));
            }
        }

        config.reserve(lines.size());
        std::for_each(std::move_iterator(begin(lines)), std::move_iterator(end(lines)), [&](std::string &&str){
            const std::string delimiter = "=";
            const uint32_t pos = str.find(delimiter);

            std::string first = str.substr(0, pos);  //first contains the first part.
            str.erase(0, pos+delimiter.length()); //str contains the second part.
            trim(first);
            trim(str);
            config.emplace(std::move(first), std::move(str));
        });

        setConfig(config);
        return config;
    } else
    {
        std::cerr << "Unable to find file: " << file << std::endl;
        std::exit(1);
    }

}



void MessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    QByteArray localMsg = msg.toLocal8Bit();
    switch (type) {
        case QtDebugMsg:
            fprintf(stdout, "\x1b[32m" "[DEBUG]: %s \033[30m(%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtInfoMsg:
            fprintf(stdout, "\x1b[33m" "[INFO]: %s \033[30m(%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtWarningMsg:
            fprintf(stdout, "\x1b[33m" "[WARNING]: %s \033[30m(%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtCriticalMsg:
            fprintf(stderr, "\x1b[31m" "[CRITICAL]: %s \033[30m(%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            break;
        case QtFatalMsg:
            fprintf(stderr, "\x1b[31m" "[FATAL]: %s \033[30m(%s:%u, %s)\n", localMsg.constData(), context.file, context.line, context.function);
            abort();
    }
}

template<typename T>
concept QApp  = requires(T a) {
    std::is_base_of_v<QCoreApplication, T>;
};

template<QApp T>
class QRobocompApp : public T
{
public:
    QRobocompApp(int &argc, char **argv) : T(argc, argv) {}

    bool notify(QObject* receiver, QEvent* event) override
    {
        try {
            return QApplication::notify(receiver, event);
        } catch (const std::exception &e) {
            std::cerr << "Exception captured by QRobocompApp notify handler \n"  << e.what() << std::endl;
        } catch (...) {
            auto ptr = std::current_exception();
            int status = 0;
            char *buffer = __cxxabiv1::__cxa_demangle(abi::__cxa_current_exception_type()->name(), nullptr,
                                                      nullptr, &status);
            if (!buffer) {
                std::cerr << "Unknown exception captured by QRobocompApp notify handler"<< std::endl;;
            } else {
                std::string demangled = buffer;
                std::free(buffer);
                std::cerr << "Exception captured by QRobocompApp notify handler \n"  << demangled<< std::endl;;
            }
        }
        return false;
    }

    //bool exit_on_signal_exception = true; A lo mejor se podría usar esto.
};

int main(int argc, char* argv[])
{



    //////////////////////////////////////////////////
    //// Read config file
    //////////////////////////////////////////////////
	// Set config file
	std::string configFile{"etc/config"};
	if (argc > 1)
	{
        configFile = argv[1];
        std::cout << "Using file " << configFile << std::endl;

    } else {
	    std::cout << "Warning: using default file etc/config" << std::endl;
	}
	//::idserver app(prefix, startup_check_flag);

    std::unordered_map<std::string, std::string> config = parseConfig(configFile);

    //////////////////////////////////////////////////
    //// Signal Handler
    //////////////////////////////////////////////////
    sigset_t sigs;
    sigemptyset(&sigs);
    sigaddset(&sigs, SIGHUP);
    sigaddset(&sigs, SIGINT);
    sigaddset(&sigs, SIGTERM);
    sigprocmask(SIG_UNBLOCK, &sigs, nullptr);

    UnixSignalWatcher sigwatch; //This want to be launched with a Qthread.
    sigwatch.watchForSignal(SIGINT);
    sigwatch.watchForSignal(SIGTERM);

    int status=EXIT_SUCCESS;

#ifdef USE_ICE

    //////////////////////////////////////////////////
    //// Initialize Qt objects
    //////////////////////////////////////////////////

    Ice::CommunicatorPtr communicator = Ice::initialize(argc, argv);
    for (const auto &[k,v] : config)
    {
        communicator->getProperties()->setProperty(k, v);
    }


    std::string tmp;
    try {

        // Server adapter creation and publication
        //if (not configGetString(config, "", "NAME", tmp , "")) {
        //    cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy NAME\n";
        //}
        //Ice::ObjectAdapterPtr adapterCommonBehavior = communicator->createObjectAdapterWithEndpoints("ENDPOINT_NAME", tmp);
        //auto commonbehaviorI = std::make_shared<CommonBehaviorI>(monitor);
        //adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("ENDPOINT_NAME"));
        //adapterCommonBehavior->activate();
    }
    catch(const Ice::Exception& ex)
    {
        status = EXIT_FAILURE;

        cout << "[" << PROGRAM_NAME << "]: Exception raised while creating NAME adapter: " << endl;
        cout << ex;

    }


#endif
    auto tprx = std::tuple<>();


    //////////////////////////////////////////////////
    //// Initialize Qt objects
    //////////////////////////////////////////////////
    qInstallMessageHandler(MessageHandler);

#ifdef USE_QTGUI
    QRobocompApp<QApplication> qt_app(argc, argv);  // GUI application
#else
    QRobocompApp<QCoreApplication> qt_app(argc, argv);  // NON-GUI application
#endif

    QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &qt_app, SLOT(quit()));

    bool startup_check_flag = false;


    SpecificWorker *worker = new SpecificWorker(tprx, startup_check_flag);
    QObject::connect(worker, SIGNAL(kill()), &qt_app, SLOT(quit()));


    auto period = 100;
    worker->setParams(config);
    worker->initialize(period);



    // Server adapter creation and publication
    std::cout << SERVER_FULL_NAME " started" << std::endl;

        // User defined QtGui elements ( main window, dialogs, etc )

#ifdef USE_QTGUI
        //ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
    qt_app.setQuitOnLastWindowClosed( true );
#endif
        // Run QT Application Event Loop
    qt_app.exec();


    status = EXIT_SUCCESS;

#ifdef USE_QTGUI
    qt_app.quit();
#endif

    status = EXIT_SUCCESS;

    delete worker;
    return status;

}
