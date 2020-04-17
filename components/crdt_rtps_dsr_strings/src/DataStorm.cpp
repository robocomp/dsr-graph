//
// Created by crivac on 20/12/18.
//


#include "DataStorm.h"

std::ostream& operator<<(std::ostream &os, DataStorm::SampleEvent sampleType) {
	switch (sampleType) {
	    case DataStorm::SampleEvent::Add:
		os << "Add";
		break;
	    case DataStorm::SampleEvent::Update:
		os << "Update";
		break;
	    case DataStorm::SampleEvent::Remove:
		os << "Remove";
		break;
	    case DataStorm::SampleEvent::PartialUpdate:
		os << "PartialUpdate";
		break;
	    default:
		os << static_cast<int>(sampleType);
		break;
	}
return os;
}

std::ostream& operator<<(std::ostream &os, const std::vector <DataStorm::SampleEvent> &types) {
	os << "[";
	for (auto p = types.begin(); p != types.end(); ++p) {
	    if (p != types.begin()) {
		os << ',';
	    }
		os << *p;
	}
	os << "]";

	return os;
}


