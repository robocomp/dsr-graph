//
// Created by araceli on 6/4/20.
//

#ifndef LOCALPERSON_H
#define LOCALPERSON_H

#include <genericworker.h>

typedef struct{
	int32_t id;
	float x;
	float z;
	float angle;

} localPerson;

typedef std::vector<localPerson> localPersonsVec;


#endif 