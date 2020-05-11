//
// Created by juancarlos on 7/5/20.
//

#ifndef CRDT_RTPS_DSR_STRINGS_CRDT_UNIT_TEST_H
#define CRDT_RTPS_DSR_STRINGS_CRDT_UNIT_TEST_H

#include "../../../../graph-related-classes/CRDT.h"
#include "DSR_test.h"


class CRDT_G_api_test {
    public:
        CRDT_G_api_test(){ };

        CRDT_G_api_test(const shared_ptr<DSR_test> t, const std::string& file1, const std::string& file2)
        : DSR_t(t) , empty_file (file1), test_file (file2){};

        void test(const shared_ptr<CRDT::CRDTGraph>& G);

    private:
        shared_ptr<DSR_test> DSR_t;
        std::string empty_file;
        std::string test_file;
        std::chrono::steady_clock::time_point start, end;
};


#endif //CRDT_RTPS_DSR_STRINGS_CRDT_UNIT_TEST_H
