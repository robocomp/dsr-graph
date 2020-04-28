# dsr-graph
Development of new DSR shared graph

To install this component, 

* Clone https://github.com/ryanhaining/cppitertools in /usr/local/include
* Install version 9 of g++
* Install the middleware Fast-RTPS de eProsima 

git clone https://github.com/eProsima/Fast-CDR.git 

En el fichero Fast-CDR/include/fastcdr/Cdr.h , línea 2146. Cambiar la función deserialize por esta otra (es la misma solo que Key y Value se definen dentro del bucle) :

 template<class _K, class _T>
                    Cdr& deserialize(std::map<_K, _T> &map_t)
                    {
                        uint32_t seqLength = 0;
                        state state_(*this);

                        *this >> seqLength;

                        try
                        {
                            //map_t.resize(seqLength);
                            for (uint32_t i = 0; i < seqLength; ++i)
                            {
                                _K key;
                                _T value;
                                *this >> key;
                                *this >> value;
                                map_t.emplace(std::pair<_K, _T>(key, value));
                            }
                            //return deserializeArray(vector_t.data(), vector_t.size());
                        }
                        catch(eprosima::fastcdr::exception::Exception &ex)
                        {
                            setState(state_);
                            ex.raise();
                        }

                        return *this;
                    }


 mkdir Fast-CDR/build && cd Fast-CDR/build
 cmake ..
 cmake --build . --target install

* Reinstalar fastrtps

git clone https://github.com/eProsima/Fast-RTPS.git 

mkdir Fast-RTPS/build && cd Fast-RTPS/build

cmake ..

cmake --build . --target install
