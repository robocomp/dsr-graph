# dsr-graph
Development of new DSR shared graph

To install this component, 

* Clone https://github.com/ryanhaining/cppitertools in /usr/local/include
* Install version 9 of g++ (https://askubuntu.com/questions/1140183/install-gcc-9-on-ubuntu-18-04/1149383#1149383)
* Install the middleware Fast-RTPS de eProsima manually

    *  https://github.com/eProsima/Fast-RTPS#manual-installation
    
    You will need three libs: Fast CDR, Foonathan memory and Fast RTPS in this order.

    *  En el fichero Fast-CDR/include/fastcdr/Cdr.h , línea 2146. Cambiar la función deserialize por esta otra (es la misma solo que Key y Value se definen dentro del bucle) :
```
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
```
Compile and install:
```bash
mkdir Fast-CDR/build && cd Fast-CDR/build
cmake ..
cmake --build . --target install
```

*  Install Foonathan Memory:
    * https://github.com/foonathan/memory
    * https://github.com/eProsima/Fast-RTPS/issues/620#issuecomment-525274544
```
git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build && cd build
cmake ..
cmake --build . --target install
  ```

* Install dependencies:
```
sudo apt install libasio-dev/bionic libtinyxml2-dev/bionic
```

*  Install Fast rtps 
```bash
git clone https://github.com/eProsima/Fast-RTPS.git

mkdir Fast-RTPS/build && cd Fast-RTPS/build

cmake ..

cmake --build . --target install
```