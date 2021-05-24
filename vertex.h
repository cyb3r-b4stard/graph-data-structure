#ifndef VERTEX_H
#define VERTEX_H

template <typename V>
class Vertex {
public:
    int id;
    V   value;

    Vertex(int _id, V _value) : id(_id), value(_value) {}
    Vertex() = default;

    bool operator==(const Vertex &vertex) {
        return (this->value == vertex.value && this->id == vertex.id);
    }
};

#endif