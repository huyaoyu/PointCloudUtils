//
// Created by yaoyu on 5/24/20.
//

#include <iostream>
#include <string>
#include <vector>

#define SHOW_ARRAY(array, n) \
    show_array(array, n, #array);

#define SHOW_SEQ(seq) \
    show_seq(seq, #seq);

template < typename T >
static void show_array( const T *array, int n, const std::string &name ) {
    std::cout << name << " " << array << ": ";
    for ( int i = 0; i < n; ++i ) {
        std::cout << &array[i] << " " << array[i] << ", ";
    }
    std::cout << "\n";
}

template < typename T >
static void show_seq( const T &seq, const std::string &name ) {
    std::cout << name << ": \n";
    for( const auto& v : seq ) {
        std::cout << v << ", ";
    }
    std::cout << "\n";
}

class Value {
public:
    Value() = default;

    Value(int v)
        : val{v}
    {}

    Value( const Value &other ) {
        val = other.val;
    }

    Value ( Value &&other ) noexcept {
        if ( flag ) {
            std::cout << "Move constructor. " << std::endl; // Force flush output.
        }
        val = other.val;
    }

    ~Value() = default;

    Value& operator= ( const Value &other ) {
        if ( flag ) {
            std::cout << "Copy assignment. " << std::endl; // Force flush output.
        }
        this->val = other.val;
        return *this;
    }

    Value& operator= ( Value &&other ) noexcept {
        if ( flag ) {
            std::cout << "Move assignment. " << std::endl; // Force flush output.
        }
        this->val = other.val;
        return *this;
    }

    friend std::ostream& operator<< ( std::ostream &out, const Value &value ) {
        out << "val = " << value.val;
        return out;
    }

public:
    int val;

public:
    static bool flag;
};

bool Value::flag = false;

static Value create_value(int v) {
    Value temp(v);
    std::cout << "&temp = " << &temp << "\n";
    return temp;
}

static std::vector<Value> create_values() {
    std::vector<Value> valuesLocal;
    valuesLocal.emplace_back( 0 );
    valuesLocal.emplace_back( 1 );

    std::cout << "&valuesLocal = " << &valuesLocal << "\n";
    std::cout << "valuesLocal.data() = " << valuesLocal.data() << "\n";
    std::cout << "&valuesLocal[0] = " << &valuesLocal[0] << "\n";
    std::cout << "&valuesLocal[1] = " << &valuesLocal[1] << "\n";

    return valuesLocal;
}

int main( int argc, char **argv ) {
    std::cout << "Hello, TryMoveSemantics! \n";

    {
        std::cout << "========== Swap 2 plain arrays. ==========\n";
        int array0[2] { 0, 1 };
        int array1[2] { 2, 3 };
        SHOW_ARRAY(array0, 2)
        SHOW_ARRAY(array1, 2)

        std::swap( array0, array1 );

        SHOW_ARRAY(array0, 2)
        SHOW_ARRAY(array1, 2)
        std::cout << "\n";
    }

    {
        std::cout << "========== Swap 2 plain arrays of objects. ==========\n";
        Value array0[2] { 0, 1 };
        Value array1[2] { 2, 3 };

        SHOW_ARRAY(array0, 2)
        SHOW_ARRAY(array1, 2)

        std::swap( array0, array1 );

        SHOW_ARRAY(array0, 2)
        SHOW_ARRAY(array1, 2)
        std::cout << "\n";
    }

    {
        std::cout << "========== Return std::vector by move. ==========\n";
        Value::flag = true;
        std::cout << "Creation and initialization. \n";
        std::vector<Value> values = create_values();
        std::cout << "&values = " << &values << "\n";
        std::cout << "values.data() = " << values.data() << "\n";
        std::cout << "&values[0] = " << &values[0] << "\n";
        std::cout << "&values[1] = " << &values[1] << "\n";
        SHOW_SEQ(values)
        Value::flag = false;
        std::cout << "\n";
    }

    {
        std::cout << "========== Separate creation and assignment. ==========\n";
        Value::flag = true;
        std::cout << "Separated creation and initialization. \n";
        std::vector<Value> values {2, 3};
        SHOW_SEQ(values)
        values = create_values();
        std::cout << "&values = " << &values << "\n";
        std::cout << "values.data() = " << values.data() << "\n";
        std::cout << "&values[0] = " << &values[0] << "\n";
        std::cout << "&values[1] = " << &values[1] << "\n";
        SHOW_SEQ(values)

        std::vector<Value> valuesLong {2, 3, 4};
        std::cout << "&valuesLong = " << &valuesLong << "\n";
        std::cout << "valuesLong.data() = " << valuesLong.data() << "\n";
        SHOW_SEQ(valuesLong)
        valuesLong = create_values();
        std::cout << "&valuesLong = " << &valuesLong << "\n";
        std::cout << "valuesLong.data() = " << valuesLong.data() << "\n";
        std::cout << "&valuesLong[0] = " << &valuesLong[0] << "\n";
        std::cout << "&valuesLong[1] = " << &valuesLong[1] << "\n";
        std::cout << "valuesLong.size() = " << valuesLong.size() << "\n";
        SHOW_SEQ(values)
        Value::flag = false;
        std::cout << "\n";
    }

    {
        std::cout << "========== Assignment by move. ==========\n";
        Value::flag = true;
        Value v0(0);
        Value v1(1);
        std::cout << "v0: " << v0 << ", "
                  << "v1: " << v1 << "\n";

        v0 = std::move(v1);
        std::cout << "v0: " << v0 << ", "
                  << "v1: " << v1 << "\n";

        v0 = create_value(2);
        std::cout << "v0: " << v0 << "\n";

        std::cout << "Initialization assignment. \n";
        Value v2 = create_value(3);
        std::cout << "&v2 = " << &v2 << "\n";
        std::cout << "v2: " << v2 << "\n";

        Value::flag = false;
        std::cout << "\n";
    }

    return 0;
}