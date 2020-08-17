//
// Created by juancarlos on 6/8/20.
//

#ifndef TYPE_CHECKER_H
#define TYPE_CHECKER_H

#include<unordered_map>
#include<string_view>
#include<functional>
#include<any>
#include<typeindex>



class TYPES {
public:
    static std::unordered_map<std::string_view, std::function<bool(const std::any&)>> map_fn_;

    static bool REGISTER(std::string_view s, const std::any& type ) {
        map_fn_.emplace(std::make_pair(s, [s, t = std::type_index(type.type()) ](const std::any &el) -> bool {
            //std::cout << t.name() <<  "  " <<  std::type_index(el.type()).name() << std::endl;
            return t == std::type_index(el.type());
        }));
        return true;
    }
    static bool CHECKTYPE(std::string_view s, const std::any& val) {
        if (map_fn_.find(s) != map_fn_.end()) {
            return map_fn_.at(s)(val);
        } else  {
            REGISTER(s, val);
            return map_fn_.at(s)(val);
        }
    }
};


#endif //TYPE_CHECKER_H
