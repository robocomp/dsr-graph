//
// Created by juancarlos on 31/7/20.
//

#ifndef IDSERVER_DSR_ATTR_NAME_H
#define IDSERVER_DSR_ATTR_NAME_H



template<typename Va>
static bool constexpr allowed_types = std::is_same<std::int32_t, Va>::value ||
                                      std::is_same<std::uint32_t, Va>::value ||
                                      std::is_same<std::string_view, Va>::value ||
                                      std::is_same<std::string, Va>::value ||
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
                             \

REGISTER_TYPE(pos_x, float);
REGISTER_TYPE(level, int);
REGISTER_TYPE(name, const std::string&);
REGISTER_TYPE(parent, std::uint32_t);
REGISTER_TYPE(rotation_euler_xyz, std::reference_wrapper<std::vector<float>>);
REGISTER_TYPE(translation, std::reference_wrapper<std::vector<float>>);
REGISTER_TYPE(color, std::reference_wrapper<std::string>);
REGISTER_TYPE(texture, std::reference_wrapper<std::string>);



#define CREATE_MAP() \


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
