//
// Created by starwalker on 18.08.2021.
//

#include "VideoPipelineStruct/VideoPipelineStruct.hpp"

VideoPipelineStruct::VideoPipelineStruct(size_t size) {
    this->_data = new char[size + 8];
    *((size_t *) this->_data) = size;
}

VideoPipelineStruct::~VideoPipelineStruct() {
    delete[] this->_data;
}

size_t VideoPipelineStruct::Size() {
    return *((size_t *) this->_data);
}

char *VideoPipelineStruct::Begin() {
    return this->_data;
}

char *VideoPipelineStruct::StringBegin() {
    return this->_data + sizeof(size_t);
}
