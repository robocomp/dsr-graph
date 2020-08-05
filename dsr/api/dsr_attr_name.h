//
// Created by juancarlos on 31/7/20.
//

#ifndef IDSERVER_DSR_ATTR_NAME_H
#define IDSERVER_DSR_ATTR_NAME_H

#include <concepts>



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


#endif //IDSERVER_DSR_ATTR_NAME_H
