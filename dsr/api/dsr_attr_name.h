//
// Created by juancarlos on 31/7/20.
//

#ifndef DSR_ATTR_NAME_H
#define DSR_ATTR_NAME_H


#include <typeindex>
#include <tuple>
#include <cstdint>
#include <string>
#include <vector>
#include <type_traits>
#include <functional>
#include <unordered_map>
#include <any>
#include <cmath>
#include <memory>
#include "../core/types/crdt_types.h"
#include "../core/types/user_types.h"
#include "../core/types/type_checker.h"
#include <qmat/QMatAll>

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



//Comprueba si en el tipo T existen los attributos attr_type y attr_name
template <typename, typename = void, typename = void>
struct is_attr_name : std::false_type {};
template <typename T>
struct is_attr_name<T, std::void_t<decltype(T::attr_type), decltype(T::attr_name)>, typename std::enable_if<T::attr_type >::type > : std::true_type {};


template<typename T>
struct is_reference_wrapper : false_type {};
template<typename T>
struct is_reference_wrapper<reference_wrapper<T>> : true_type{};

template<typename name, class Ta>
static constexpr bool valid_type ()
{
    if constexpr(is_reference_wrapper<decltype(name::type)>::value) {
        using ref_type = typename decltype(name::type)::type;
        using Selected_Type = std::remove_reference_t<std::remove_cv_t<ref_type>>; // g++10 da error con ice, no podemos usar std::remove_cv_ref
        return std::is_same_v<Selected_Type, std::remove_cv_t<std::remove_reference_t<Ta>>>;
    } else {
        using Selected_Type = std::remove_reference_t<std::remove_cv_t<decltype(name::type)>>; // g++10 da error con ice, no podemos usar std::remove_cv_ref
        return std::is_same_v<Selected_Type, std::remove_cv_t<std::remove_reference_t<Ta>>>;
    }
}


// Attributes
//Define el tipo utilizado para validar los tipos de atributos durante la compilación
template<const std::string_view& n, typename Tn>
struct Attr {
    static constexpr bool attr_type = bool_constant<allowed_types<Tn>>(); //Para comprobar el tipo
    static constexpr std::string_view attr_name = std::string_view(n); //Nombre del atributo usado en el mapa.
    static Tn type; //tipo que se devuelve de G.
};


#define REGISTER_FN(x, it)  \
                            inline bool x ##_b =  TYPES::REGISTER( x##_str, []<typename tp = it>() -> auto {\
                                if constexpr (is_reference_wrapper<tp>::value) {                           \
                                    using tp_c = std::remove_const_t<typename tp::type>;                                      \
                                    return tp_c();\
                                } else {\
                                    static_assert(std::is_constructible_v<tp>, #x "is not constructible without arguments, define your function manually");\
                                    return tp();                                                              \
                                }\
                            }() );     \
                            \


#define REGISTER_TYPE(x, ot) \
                            static constexpr auto    x ##_str = std::string_view(#x ); \
                            using x ##_att = Attr< x##_str, ot>;                        \
                            REGISTER_FN(x, ot) \
                            \



inline std::unordered_map<std::string_view, std::function<bool(const std::any&)>> TYPES::map_fn_;


REGISTER_TYPE(pos_x, float);
REGISTER_TYPE(pos_y, float);
REGISTER_TYPE(level, int);
REGISTER_TYPE(name, std::reference_wrapper<const std::string>);
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
REGISTER_TYPE(base_target_x, float);
REGISTER_TYPE(base_target_y, float);

#endif //DSR_ATTR_NAME_H
