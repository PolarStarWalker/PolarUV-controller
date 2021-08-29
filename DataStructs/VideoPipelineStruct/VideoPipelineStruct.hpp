#ifndef CLIENT_VIDEOPIPELINESTRUCT_HPP
#define CLIENT_VIDEOPIPELINESTRUCT_HPP

class VideoPipelineStruct {
private:
    char *_data;

public:
    VideoPipelineStruct(size_t size);
    ~VideoPipelineStruct();
    size_t Size();
    char* Begin();
    char* StringBegin();
};

#endif
