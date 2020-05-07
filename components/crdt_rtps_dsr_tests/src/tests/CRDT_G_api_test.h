//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_STRINGS_CRDT_UNIT_TEST_H
#define CRDT_RTPS_DSR_STRINGS_CRDT_UNIT_TEST_H

#include "Test_utils.h"
#include "../../../../graph-related-classes/CRDT.h"


class CRDT_G_api_test {
    public:
        CRDT_G_api_test()  { };

        CRDT_G_api_test(const shared_ptr<Test_utils> t, const std::string& file1, const std::string& file2)
        : testutils(t) , empty_file (file1), test_file (file2){};

        void test(const shared_ptr<CRDT::CRDTGraph>& G);

    private:


        shared_ptr<Test_utils> testutils;
        std::string empty_file;
        std::string test_file;
        std::chrono::steady_clock::time_point start, end;
        /*
        * Operaciones básicas
        * */

        //Obtener un nodo que no existe.
        void get_nonexistent_node(const shared_ptr<CRDT::CRDTGraph>& G);
        //Obtener un edge que no existe.
        void get_nonexistent_edge(const shared_ptr<CRDT::CRDTGraph>& G);
        //Añadir nodo.
        void insert_node(const shared_ptr<CRDT::CRDTGraph>& G);
        //Obtener un nodo que existe
        void get_existent_node(const shared_ptr<CRDT::CRDTGraph>& G);
        //Reemplazar un nodo.
        void update_node(const shared_ptr<CRDT::CRDTGraph>& G);
        //Añadir un edge.
        void insert_edge(const shared_ptr<CRDT::CRDTGraph>& G);
        //Obtener un edge que existe
        void get_existent_edge(const shared_ptr<CRDT::CRDTGraph>& G);
        //Reemplazar un edge.
        void update_edge(const shared_ptr<CRDT::CRDTGraph>& G);
        //Borrar un edge.
        void delete_edge(const shared_ptr<CRDT::CRDTGraph>& G);
        //Borrar un edge que no existe.
        void delete_nonexistent_edge(const shared_ptr<CRDT::CRDTGraph>& G);
        //Borrar nodo que existe.
        void delete_node(const shared_ptr<CRDT::CRDTGraph>& G);
        //Borrar nodo que no existe.
        void delete_nonexistent_node(const shared_ptr<CRDT::CRDTGraph>& G);

        /*
         * Carga de ficheros
         * */
        //Cargar fichero vacío.
        void load_empty_file(const shared_ptr<CRDT::CRDTGraph>& G, const std::string& File);
        //Cargar fichero con contenido
        void load_file(const shared_ptr<CRDT::CRDTGraph>& G, const std::string& File);

};


#endif //CRDT_RTPS_DSR_STRINGS_CRDT_UNIT_TEST_H
