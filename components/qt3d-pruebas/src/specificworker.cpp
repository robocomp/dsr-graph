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
#include <qt3dwindow.h>
#include "specificworker.h"

#include <opencv4/opencv2/rgbd.hpp>

template <typename T>
struct hash_eigen {
    std::size_t operator()(T const& matrix) const {
        size_t seed = 0;
        for (int i = 0; i < (int)matrix.size(); i++) {
            auto elem = *(matrix.data() + i);
            seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 +
                    (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

static std::tuple<std::vector<Mat::Vector3d>, Mat::Vector3d, Mat::Vector3d> get_points_pairs(const std::vector<std::tuple<float, float, float>> & vert, size_t i)
{
    auto [x1, y1, z1] = vert[i];
    auto [x2, y2, z2] = vert[i + 1];
    auto [x3, y3, z3] = vert[i + 2];
    auto [x4, y4, z4] = vert[i + 3];
    auto [x5, y5, z5] = vert[i + 4];
    auto [x6, y6, z6] = vert[i + 5];

    auto [min_x,max_x] = std::minmax({x1, x2, x3, x4, x5, x6});
    auto [min_y,max_y] = std::minmax({y1, y2, y3, y4, y5, y6});
    auto [min_z,max_z] = std::minmax({z1, z2, z3, z4, z5, z6});

    min_x = std::round(min_x);
    min_y = std::round(min_y);
    min_z = std::round(min_z);
    max_x = std::round(max_x);
    max_y = std::round(max_y);
    max_z = std::round(max_z);

    auto min = Mat::Vector3d(min_x*10, min_y*10, min_z*10);
    auto max = Mat::Vector3d(max_x*10, max_y*10, max_z*10);
    auto p3 = Mat::Vector3d(max_x*10, max_y*10, min_z*10);
    auto p4 = Mat::Vector3d(max_x*10, min_y*10, max_z*10);
    auto p5 = Mat::Vector3d(min_x*10, max_y*10, max_z*10);
    auto p6 = Mat::Vector3d(min_x*10, min_y*10, max_z*10);
    auto p7 = Mat::Vector3d(min_x*10, max_y*10, min_z*10);
    auto p8 = Mat::Vector3d(max_x*10, min_y*10, min_z*10);


    std::unordered_set<Mat::Vector3d, hash_eigen<Mat::Vector3d>> points_s {min, max, p3, p4, p5, p6, p7, p8};
    std::vector<Mat::Vector3d> points( points_s.begin(), points_s.end());
    std::erase_if(points, [&](auto &a) { return a == min; } );
    std::erase_if(points, [&](auto &a) { return a == max; });

    return {points, min, max};
}


//Calculate segments inside img bounds.
//https://stackoverflow.com/a/563275
//https://stackoverflow.com/a/1201356
static void pair_points_inside_img(std::unique_ptr<DSR::CameraAPI>& cam,  const Mat::Vector2d& p1, const Mat::Vector2d& p2,
                            std::vector<Mat::Vector2d>& ps_vec,
                            std::vector<std::pair<Mat::Vector2d, Mat::Vector2d>>& lines)
{

    auto is_inside_img = [&] (const Mat::Vector2d& p) -> bool
    {
        return (p.x() <= cam->get_width() and p.y() <= cam->get_height() and p.x() >= 0 and p.y() >= 0);
    };

    bool inside_1 = is_inside_img(p1), inside_2 = is_inside_img(p2);

    auto img_p0 = Mat::Vector2d(0,0);
    auto img_p1 = Mat::Vector2d(cam->get_width(),0);
    auto img_p2 = Mat::Vector2d(0,cam->get_height());
    auto img_p3 = Mat::Vector2d(cam->get_width(),cam->get_height());


    auto compute = [] (const  Mat::Vector2d& a, const  Mat::Vector2d& b, const Mat::Vector2d& c, const Mat::Vector2d& d) -> double {

        // E = B-A = ( Bx-Ax, By-Ay )
        // F = D-C = ( Dx-Cx, Dy-Cy )
        // P = ( -Ey, Ex )

        const Mat::Vector2d e { b.x()-a.x(), b.y()-a.y() };
        const Mat::Vector2d f  { d.x()-c.x(), d.y()-c.y() };
        const Mat::Vector2d p  {-e.y(),  e.x()};

        // h = ( (A-C) * P ) / ( F * P )
        const auto intersection = f.x()*p.x()+f.y()*p.y();
        if(intersection == 0) { // Paralel lines
            return std::numeric_limits<double>::quiet_NaN();
        }
        return ( (a.x() - c.x()) * p.x() + (a.y() - c.y()) * p.y()) / intersection;

    };

    auto compute_intersection = [&] (const Mat::Vector2d& a, const  Mat::Vector2d& b, const  Mat::Vector2d& c, const  Mat::Vector2d& d) -> std::optional<Mat::Vector2d> {
        double h1 = compute(a, b, c, d);
        double h2 = compute(c, d, a, b);
        const bool parallel = (h1 == std::numeric_limits<double>::quiet_NaN()) or
                              (h2 == std::numeric_limits<double>::quiet_NaN());

        const Mat::Vector2d f {d.x()-c.x(), d.y()-c.y() };
        auto x =  c.x() + f.x() * h1;
        auto y =  c.y() + f.y() * h1;
        if (parallel) return {};
        if (h1 >= 0 && h1 <= 1 && h2 >= 0 && h2 <= 1) return Mat::Vector2d(x, y);
        else return {};

    };


    int cnt = 0;
    if (not inside_1 or not inside_2)
    {
        std::cout << "--------------" << std::endl;
        std::unordered_set<Mat::Vector2d, hash_eigen<Mat::Vector2d>> ps_s; //use a set to avoid dupes in (0,0) (512, 512), etc.
        if (auto p = compute_intersection(p1, p2 , img_p0, img_p1); p.has_value()) { std::cout << "[ (" << p1.x() << " " << p1.y() << ") (" << p2.x() << " " << p2.y()  << "] cortan  con horizontal arriba " << std::endl; cnt++; ps_s.insert(*p); }
        if (auto p = compute_intersection(p1, p2 , img_p0, img_p2); p.has_value()) { std::cout << "[ (" << p1.x() << " " << p1.y() << ") (" << p2.x() << " " << p2.y()  << "] cortan  con vertical izquierda " << std::endl; cnt++; ps_s.insert(*p); }
        if (auto p = compute_intersection(p1, p2 , img_p1, img_p3); p.has_value()) { std::cout << "[ (" << p1.x() << " " << p1.y() << ") (" << p2.x() << " " << p2.y()  << "] cortan con vertical derecha  ("<< p->x() << " " << p->y() << ")" << std::endl; cnt++; ps_s.insert(*p); }
        if (auto p = compute_intersection(p1, p2 , img_p2, img_p3); p.has_value()) {  std::cout << "[ (" << p1.x() << " " << p1.y() << ") (" << p2.x() << " " << p2.y()  << "] cortan con horizontal abajo ("<< p->x() << " " << p->y() << ")" << std::endl; cnt++; ps_s.insert(*p); }

        std::vector<Mat::Vector2d> ps (ps_s.begin(), ps_s.end());
        //std::cout << cnt << std::endl;

        if (not inside_1 and not inside_2 and cnt == 2 and is_inside_img(ps[0]) and is_inside_img(ps[1])) {
            ps_vec.push_back(ps[0]);
            ps_vec.push_back(ps[1]);
            lines.emplace_back(std::make_pair(ps[0], ps[1]));
        } else if (auto b1 = (not inside_1 and inside_2), b2 = (inside_1 and not inside_2); b1 or b2)
        {
            auto& p = (b1) ? p1 : p2;
            auto& p_ = (b1) ? p2 : p1;

            if (cnt > 1)
            {
                //nearest in ps to p
                Mat::Vector2d p_nearest =  *std::min_element(ps.begin(), ps.end(), [&](const Mat::Vector2d& x1, const Mat::Vector2d& x2)
                {
                    auto dist = [&](const Mat::Vector2d& po) { return std::hypot(po.x() - p.x(), po.y() - p.y()); };
                    return dist(x1) < dist(x2);
                });
                /*
                for (const auto& p_n : ps) {
                    lines.emplace_back(std::make_pair(p_n, p_));
                    ps_vec.push_back(p_n);
                    ps_vec.push_back(p_);
                }*/

            } else if (cnt == 1) {

                (b1) ? lines.emplace_back(std::make_pair(p2,ps[0])) : lines.emplace_back(std::make_pair(ps[0], p1));
                ps_vec.push_back(p);
                ps_vec.push_back(ps[0]);
            }
        }

    } else {
        lines.emplace_back(p1, p2);
        ps_vec.push_back(p1);
        ps_vec.push_back(p2);
    }
}
/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }





	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);

	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";

	return true;
}


void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//cam = G->get_camera_api(G->get_nodes_by_type("RGB").at(0));
		inner = G->get_inner_eigen_api();

		//dsr update signals
		//connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		//connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);


		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, 0 , main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        world_view = std::make_unique<World>(G.get());
        graph_viewer->add_custom_widget_to_dock("PONER NOMBRE AQUÍ", world_view->getWidget());
        world_view->show();


        this->Period = period;
		timer.start(Period);
	}

}
void SpecificWorker::add_or_assign_edge_slot(std::uint64_t from, std::uint64_t to,  const std::string &type)
{
    //TODO: hacer aquí cosas

    //std::cout << type << std::endl;
    if (type == "attention_action") {
    const std::vector <std::string> state_names = {
            "within",
            "covered_by",
            "overlaps",
            "intersects",
            "contains",
            "disjoint",
            "over",
            "under"
    };
    //std::cout << "HOLA" << std::endl;
    std::string name = G->get_name_from_id(to).value_or("");
    auto pos = inner->get_translation_vector("world", name);
    if (pos.has_value()) {
        std::cout << "objeto visible" << std::endl;
        std::tuple<float, float, float> point((*pos).x() / 10.0, (*pos).y() / 10.0, (*pos).z() / 10.0);
        world_view->insert_point(point);
        std::vector <std::pair<std::string, float>> near = geom.nearest(point, 1);

        if (!cam) cam = G->get_camera_api(G->get_nodes_by_type(rgbd_type_str.data()).at(0));


        std::vector<uint8_t> img = cam->get_rgb_image().value();
        auto* rgb_data = img.data(); //new unsigned char (img.size());
        //std::memcpy(rgb_data, img.data(), sizeof(uint8_t)*img.size());
        cv::Mat rgb (cam->get_height(), cam->get_width(), CV_8UC3, rgb_data);


        if (!contornos.contains(name)) {

            auto rgbd_label = new QLabel();
            rgbd_label->setWindowTitle(QString(name.data()) + " contorno");
            rgbd_label->show();
            rgbd_label->setMinimumSize(cam->get_height(), cam->get_width());
            contornos[name] = rgbd_label;
            auto *m = new cv::Mat;
            *m = cv::Mat::ones(cam->get_height(), cam->get_width(), CV_8UC3);
            contornos_img[name] = std::unique_ptr<cv::Mat>(m);
            auto pix = QPixmap::fromImage(
                    QImage(contornos_img[name]->data, cam->get_width(), cam->get_height(),
                           QImage::Format_RGB888));
            contornos[name]->setPixmap(pix);
        }

        contornos_img[name]->setTo(255);

        Mat::Vector3d obj_cam = inner->transform( "viriato_head_camera_sensor", Mat::Vector3d((*pos).x() , (*pos).y() , (*pos).z() ), "world").value();
        Mat::Vector2d obj = cam->project(obj_cam);

        rgb.copyTo(*contornos_img[name]);


        //////////////////////////////////////////
        /// Draw cricle in detected object position.
        //////////////////////////////////////////
        cv::circle(*contornos_img[name], cv::Point((int) obj.x(), (int) obj.y()), 10,
                   cv::Scalar(255., 55., 25.), -1, -1);



        std::vector<cv::Scalar> lineColors { cv::Scalar (255., 0., 0.), cv::Scalar (0., 255., 0.), cv::Scalar (255., 0., 255.), cv::Scalar (0., 0., 255.) };
        std::vector<std::vector<std::pair<Mat::Vector2d, Mat::Vector2d>>> draw_points;
        int c = 0;

        for (auto[name_, dist] : near) {

            std::cout << "\t" << name_ << " a una distancia de " << dist << std::endl;
            auto status = geom.status(point, name_);
            std::cout << "\t" << name << " respecto a " << name_ << ":  [ ";
            for (uint8_t x : status) {
                std::cout << " " << state_names[x];
            }
            std::cout << " ]" << std::endl;

            cv::Mat gray, canny;
            cvtColor(*contornos_img[name], gray, cv::COLOR_BGR2GRAY);
            cv::blur(gray, canny, cv::Size(3, 3));
            cv::Canny(canny, canny, 100, 200, 3);
            std::vector<cv::Vec4i> lines_h;
            cv::HoughLinesP(canny, lines_h, 1, CV_PI / 180, 80, 30, 10);


            /*
            for (auto &lin : lines_h) {
                auto lin_roi = cv::minAreaRect(std::vector<cv::Point>{cv::Point(lin[0], lin[1]), cv::Point(lin[2], lin[3]) }).boundingRect();
                cv::rectangle(*contornos_img[name], lin_roi, cv::Scalar(0, 0, 255));
                //cv::line(*contornos_img[name], cv::Point(lin[0], lin[1]),
                //         cv::Point(lin[2], lin[3]), cv::Scalar(0, 0, 255), 1, 8);
            }*/

            auto[vert, idx]  = geom.get_geom_vertices_and_indices(name_);
            for (int i = 0; i < vert.size() -1; i+=6) {
                //From a group of six vertices that makes the two triangles of a mesh face remove the Hypotenuses,
                auto [points, min, max] = get_points_pairs(vert, i);
                if (points.size() == 2) {

                    auto xyz1= inner->transform("viriato_head_camera_sensor", min, "world").value();
                    auto xyz2= inner->transform("viriato_head_camera_sensor", max, "world").value();
                    auto xyz3= inner->transform("viriato_head_camera_sensor", points[0], "world").value();
                    auto xyz4= inner->transform("viriato_head_camera_sensor", points[1], "world").value();

                    std::vector<Mat::Vector2d> ps_vec;
                    std::unordered_set<Mat::Vector2d, hash_eigen<Mat::Vector2d>> ps_set;
                    std::vector<std::pair<Mat::Vector2d, Mat::Vector2d>> lines{};

                    for (auto [p1, p2] : std::vector<std::pair<Mat::Vector3d , Mat::Vector3d>>{ {xyz1, xyz3}, {xyz1, xyz4}, {xyz2, xyz3}, {xyz2, xyz4}} )
                    {

                        bool out1 = p1.y() < 0.0, out2 = p2.y() < 0.0;
                        if (out1 or out2) {
                            if (out1 and out2) continue;
                            auto v = p2 - p1;
                            auto lambda = (0.1 -p1.y()) / v.y();
                            auto x = p1.x() + lambda * v.x();
                            auto z = p1.z() + lambda * v.z();

                            Mat::Vector3d res;

                            auto between = [&](const Mat::Vector3d& a, const Mat::Vector3d& b, const Mat::Vector3d& c) {
                                const float epsilon = 0.000001;

                                auto ab = (a-b).norm();
                                auto ac = (a-c).norm();
                                auto bc = (b-c).norm();

                                //std::cout << "ab " << ab << ", ac " << ac << ", bc " << bc << ". sum = " << std::abs(ac + bc - ab) << std::endl;

                                /*  if ac+bc < ab the three points form a triangle so they are not collinear or
                                                  the three points are collinear but ac is outside the ab segment.
                                    if ac+bc == ab the three points are collinear and ac is inside the ab segment.
                                    ac+bc < ab is not a feasible result?
                                 */
                                auto res = std::abs(ac + bc - ab) < epsilon;
                                return res;
                            };

                            res[0] = x;
                            res[1] = 0.1;
                            res[2] = z;

                            if (between(p1, p2, res)) {
                                if (p1.y() < 0.0)  p1 = res;
                                else p2 = res;
                            } else continue;
                        }
                        Mat::Vector2d xy1 = cam->project(p1);
                        Mat::Vector2d xy2 = cam->project(p2);
                        //Projected points can be out of the image. Calculate the segments inside the image given start and end points.
                        cv::line((*contornos_img[name]), cv::Point(xy1.x(), xy1.y()),
                                 cv::Point(xy2.x(), xy2.y()), cv::Scalar(255, 255, 0), 2, 8);
                        pair_points_inside_img(cam, xy1, xy2, ps_vec, lines);
                    }

                    for (auto [p1, p2] : lines)
                    {
                        //continue;
                        auto roi = cv::minAreaRect(std::vector<cv::Point>{ cv::Point (p1.x(), p1.y()),cv::Point (p2.x(), p2.y()) });
                        auto roi_br = roi.boundingRect();
                        auto max_x = roi_br.x + roi_br.width;
                        auto max_y = roi_br.y + roi_br.height;
                        roi_br.x = (roi_br.x - 10 >= 0) ? roi_br.x - 10 : (( roi_br.x >= 0 ) ? roi_br.x : 0);
                        roi_br.y = (roi_br.y - 10 >= 0) ? roi_br.y - 10 : (( roi_br.y >= 0 ) ? roi_br.y : 0);
                        roi_br.width = (max_x + 10 < cam->get_width()) ? max_x + 10 - roi_br.x : (( max_x < cam->get_width() ) ? max_x - roi_br.x  : cam->get_width()-1 - roi_br.x );
                        roi_br.height = (max_y + 10 < cam->get_height()) ? max_y + 10 - roi_br.y : (( max_y < cam->get_height() ) ? max_y - roi_br.y : cam->get_height()-1 - roi_br.y);

                        //std::cout << "ROI x: " << roi_br.x << " y: " << roi_br.y << " w: " << roi_br.width << " h: "<< roi_br.height << std::endl;
                        if (roi_br.width > 0 and roi_br.height > 0) {


                            //cv::Mat reg = canny(roi_br);
                            //cvtColor(reg, reg, cv::COLOR_GRAY2RGB);
                            //std::cout << "HoughLines Lines: " << lines.size() << std::endl;
                            //cv::rectangle(*contornos_img[name], roi_br, cv::Scalar(0, 255, 255));

                            std::optional<cv::Vec<int, 4>> min_line;
                            std::optional<float> min_angle;
                            std::optional<float> min_dist;

                            for (auto &lin : lines_h) {
                                auto lin_roi = cv::minAreaRect(std::vector<cv::Point>{cv::Point(lin[0], lin[1]), cv::Point(lin[2], lin[3]) }).boundingRect();

                                /*lin_roi.x = (lin_roi.x - 10 >= 0) ? roi_br.x - 10 : (( lin_roi.x >= 0 ) ? lin_roi.x : 0);
                                lin_roi.y = (lin_roi.y - 10 >= 0) ? roi_br.y - 10 : (( lin_roi.y >= 0 ) ? lin_roi.y : 0);
                                auto lin_max_x = lin_roi.x + lin_roi.width;
                                auto lin_max_y = lin_roi.y + lin_roi.height;
                                lin_roi.width = (lin_max_x + 10 < cam->get_width()) ? lin_max_x + 10 - lin_roi.x : (( lin_max_x < cam->get_width() ) ? lin_max_x - lin_roi.x  : cam->get_width()-1 - lin_roi.x );
                                lin_roi.height = (lin_max_y + 10 < cam->get_height()) ? lin_max_y + 10 - lin_roi.y : (( lin_max_y < cam->get_height() ) ? lin_max_y - lin_roi.y : cam->get_height()-1 - lin_roi.y);
                                */


                                //std::cout << "LINE ROI x: " << lin_roi.x << " y: " << lin_roi.y << " w: " << lin_roi.width << " h: "<< lin_roi.height << std::endl;



                                if ((roi_br & lin_roi).area() > 0) { //Intersects
                                    //std::cout << "Intersección" << std::endl;
                                    //cv::line((*contornos_img[name]), cv::Point(lin[0], lin[1]),
                                    //         cv::Point(lin[2], lin[3]), cv::Scalar(0, 255, 0), 1, 8);


                                    double theta1 =  std::atan2(p1.y()- p2.y(), p1.x() - p2.y());
                                    double theta2 =  std::atan2(lin[1]- lin[3], lin[0] - lin[2]);
                                    double diff = std::abs(theta1-theta2)* 180 / M_PI;
                                    double angle= std::min(diff, std::abs(180-diff));



                                    int start_x = std::max((int)std::min(p1.x(), p2.x()), std::min(lin[0], lin[2]));
                                    int end_x = std::min((int)std::max(p1.x(), p2.x()), std::max(lin[0], lin[2]));

                                    auto m1 = (p2.y()  - p1.y())/(p2.x() - p1.x());
                                    auto n1 = p1.y() - p1.x()*m1;
                                    auto m2 = (lin[3] - lin[1] + 0.00001)/(lin[2] - lin[0] );
                                    auto n2 = lin[1] - lin[0]*m2;

                                    float dff = 0.0;

                                    if ((end_x-start_x) <= 0)
                                    {
                                        dff = std::numeric_limits<float>::infinity();
                                    } else {
                                        for (auto idx = start_x; idx <= end_x; idx++) {
                                            float y1 = m1 * idx + n1;
                                            float y2 = m2 * idx + n2;
                                            dff += std::abs(y1 - y2);
                                        }
                                        dff /= (end_x - start_x);
                                    }

                                    //std::cout << "total diff: "<<  dff  << " count " << (end_x-start_x)  << " s: " << start_x << " e:" << end_x   << std::endl;

                                    if ( (min_angle.has_value() && std::abs(angle) < *min_angle ) || !min_angle.has_value() )
                                    {
                                        min_angle = angle;
                                        min_dist = dff;
                                        min_line = lin;
                                    }

                                    //Calcular angulo entre las dos rectas. (Descartar peor a umbral.)
                                    //Comprar diferencia en las x e y en las columnas en las que ambas estan definidas.
                                }
                            }


                            if(min_line && *min_angle <= 15.f && *min_dist <= 20.f)
                            {
                                auto lin_roi = cv::minAreaRect(std::vector<cv::Point>{cv::Point((*min_line)[0], (*min_line)[1]),
                                                                                      cv::Point((*min_line)[2], (*min_line)[3]) }).boundingRect();


                                cv::Scalar col (0, 255, 0);
                                cv::line((*contornos_img[name]), cv::Point((*min_line)[0], (*min_line)[1]),
                                         cv::Point((*min_line)[2], (*min_line)[3]), col, 2, 8);
                            }
                            //reg.copyTo((*contornos_img[name])(roi_br));
                        }
                    }

                    draw_points.emplace_back(std::move(lines));
                } else{
                    std::cout << "Faltan o sobran puntos" << std::endl;
                }
            }

            c++;
        }

        c = 0;
        for (auto & draw_point : draw_points) {
            for (const auto&[ xy1, xy2 ] :  draw_point) {
                cv::circle(*contornos_img[name], cv::Point((int) xy1.x(), (int) xy1.y()), 2,
                           cv::Scalar(23., 55., 124.), -1, -1);

                cv::circle(*contornos_img[name], cv::Point((int) xy2.x(), (int) xy2.y()), 2,
                           cv::Scalar(23., 55., 124.), -1, -1);

                cv::line(*contornos_img[name], cv::Point((int) xy1.x(), (int) xy1.y()),
                         cv::Point((int) xy2.x(), (int) xy2.y()), lineColors[c]);
            }

            //c++;
        }

        auto pix = QPixmap::fromImage(
                QImage((unsigned char *) contornos_img[name]->data, cam->get_width(), cam->get_height(),
                       QImage::Format_RGB888));
        contornos[name]->setPixmap(pix);

    }

    }
};


void SpecificWorker::compute()
{
	//computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	
	
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}
