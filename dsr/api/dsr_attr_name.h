//
// Created by juancarlos on 31/7/20.
//

#ifndef IDSERVER_DSR_ATTR_NAME_H
#define IDSERVER_DSR_ATTR_NAME_H



template<typename Va>
static bool constexpr allowed_types = std::is_same<std::int32_t, Va>::value ||
                                      std::is_same<std::uint32_t, Va>::value ||
                                      std::is_same<std::string, Va>::value ||
                                      std::is_same<std::reference_wrapper<const std::string>, Va>::value ||
                                      std::is_same<std::reference_wrapper<const std::vector<float_t>>, Va>::value ||
                                      std::is_same<std::reference_wrapper<const std::vector<uint8_t>>, Va>::value ||
                                      std::is_same<std::float_t, Va>::value ||
                                      std::is_same<std::double_t, Va>::value ||
                                      std::is_same<std::vector<float_t>, Va>::value ||
                                      std::is_same<std::vector<uint8_t>, Va>::value ||
                                      std::is_same<bool, Va>::value;
template<typename Va>
static bool constexpr any_node_or_edge = std::is_same<DSR::CRDTNode, Va>::value ||
                                         std::is_same<DSR::CRDTEdge, Va>::value ||
                                         std::is_same<DSR::Node, Va>::value ||
                                         std::is_same<DSR::Edge, Va>::value
;

template<typename Va>
static bool constexpr node_or_edge = std::is_same<DSR::Node, Va>::value ||
                                     std::is_same<DSR::Edge, Va>::value
;


template<typename Va>
static bool constexpr crdt_node_or_edge = std::is_same<DSR::CRDTNode, Va>::value ||
                                     std::is_same<DSR::CRDTNode, Va>::value
;

template<typename Va>
static bool constexpr allowed_return_types = std::is_same<std::int32_t, Va>::value ||
                                             std::is_same<std::uint32_t, Va>::value ||
                                             std::is_same<std::string, Va>::value ||
                                             std::is_same<std::float_t, Va>::value ||
                                             std::is_same<std::vector<float_t>, Va>::value ||
                                             std::is_same<std::vector<uint8_t>, Va>::value ||
                                             std::is_same<bool, Va>::value ||
                                             std::is_same<QVec, Va>::value ||
                                             std::is_same<QMat, Va>::value;


// Attributes
//Define el tipo utilizado para validar los tipos de atributos durante la compilación
template<const std::string_view& n, typename Tn>
struct Attr {
    static constexpr bool attr_type = bool_constant<allowed_types<Tn>>(); //Para comprobar el tipo
    static constexpr std::string_view attr_name = std::string_view(n); //Nombre del atributo usado en el mapa.
    static Tn type; //Variable usada para acceder al tipo.
};



#define REGISTER_TYPE(x, t) \
                            static constexpr auto    x ##_str = std::string_view(#x ); \
                            using x ##_att = Attr< x##_str, t>;                        \

REGISTER_TYPE(pos_x, float);
REGISTER_TYPE(pos_y, float);
REGISTER_TYPE(level, int);
REGISTER_TYPE(name, const std::string&);
REGISTER_TYPE(parent, std::uint32_t);
REGISTER_TYPE(rotation_euler_xyz, std::reference_wrapper<const std::vector<float>>);
REGISTER_TYPE(translation, std::reference_wrapper<const std::vector<float>>);
REGISTER_TYPE(color, std::reference_wrapper<const std::string>);
REGISTER_TYPE(texture, std::reference_wrapper<const std::string>);
REGISTER_TYPE(width, int);
REGISTER_TYPE(height, int);
REGISTER_TYPE(depth, int);
REGISTER_TYPE(scalex, int);
REGISTER_TYPE(scaley, int);
REGISTER_TYPE(scalez, int);
REGISTER_TYPE(path, std::reference_wrapper<const std::string>);
REGISTER_TYPE(angles, std::reference_wrapper<const std::vector<float>>);
REGISTER_TYPE(dists, std::reference_wrapper<const std::vector<float>>);
REGISTER_TYPE(rgb, std::reference_wrapper<const std::vector<uint8_t>>);
REGISTER_TYPE(cameraID, int);
REGISTER_TYPE(focalx, int);
REGISTER_TYPE(focaly, int);
REGISTER_TYPE(alivetime, int);
REGISTER_TYPE(linear_speed, std::reference_wrapper<const std::vector<float>>);
REGISTER_TYPE(angular_speed, std::reference_wrapper<const std::vector<float>>);
REGISTER_TYPE(ref_adv_speed, float);
REGISTER_TYPE(ref_rot_speed, float);
REGISTER_TYPE(ref_side_speed, float);

/*
using att_types =
        std::variant<pos_x_att,
                    pos_y_att,
                    level_att,
                    name_att,
                    parent_att,
                    rotation_euler_xyz_att,
                    translation_att,
                    color_att,
                    texture_att,
                    width_att,
                    height_att,
                    depth_att,
                    scalex_att,
                    scaley_att,
                    scalez_att,
                    path_att,
                    angles_att,
                    dists_att,
                    rgb_att
        >;

const static  std::map<std::string_view, att_types> ATT_MAP = {
        {pos_x_str, pos_x_att()},
        {level_str, level_att()},
        {name_str, name_att()},
        {parent_str, parent_att()},
        {rotation_euler_xyz_str, rotation_euler_xyz_att ()},
        {translation_str, translation_att ()},
        {color_str, color_att ()},
        {texture_str, texture_att() },
        {width_str, width_att ()},
        {height_str, height_att ()},
        {depth_str, depth_att ()},
        {scalex_str, scalex_att()},
        {scaley_str, scaley_att()},
        {scalez_str, scalez_att ()},
        {path_str, path_att ()},
        {pos_y_str, pos_y_att ()},
        {angles_str, angles_att ()},
        {dists_str, dists_att ()},
        {rgb_str, rgb_att()}

};

template<class T = void>
T  get_type(const string& t) {
    if (t == pos_x_str) {
        return pos_x_att();
    }
    if (t == level_str) {
        return level_att();
    }
    if (t == name_str) {
        return name_att();
    }
    if (t == name_str) {
        return name_att();
    }
    if (t == name_str) {
        return name_att();
    }
    if (t == name_str) {
        return name_att();
    }
    if (t == name_str) {
        return name_att();
    }
}

*/
/*
//Variables constantes que guardan el nombre del atributo en G
static constexpr auto name_str = std::string_view("name");
static constexpr auto pos_x_str = std::string_view("pos_x");
static constexpr auto level_str = std::string_view("level");
static constexpr auto parent_str = std::string_view("parent");
static constexpr auto rotation_euler_xyz_str = std::string_view("rotation_euler_xyz");
static constexpr auto translation_str = std::string_view("translation");
static constexpr auto color_str = std::string_view("color");
static constexpr auto color_str = std::string_view("color");
static constexpr auto texture_str = std::string_view("texture");
static constexpr auto width_str = std::string_view("color");
static constexpr auto height_str = std::string_view("color");
static constexpr auto depth_str = std::string_view("color");
static constexpr auto scalex_str = std::string_view("color");
static constexpr auto scaley_str = std::string_view("color");
static constexpr auto scalez_str = std::string_view("scalez_str");
static constexpr auto color_str = std::string_view("color");
static constexpr auto color_str = std::string_view("color");



//Alias para los tipos.
using pos_x = Attr<pos_x_str, float> ;
using level = Attr<level_str, int> ;
using name = Attr<name_str, const std::string&> ;
using parent = Attr<parent_str, uint32_t>;
using rotation_xyz = Attr<rotation_euler_xyz_str, const std::vector<float>&>;
using translation = Attr<translation_str, const std::vector<float>&>;
using color = Attr<color_str, const std::string&>;
using texture = Attr<texture_str, const std::string&>

*/

#endif //IDSERVER_DSR_ATTR_NAME_H
