//
// Created by yaoyu on 4/22/20.
//

#include <iostream>
#include <memory>

class A {
public:
    A() : id(-1) {}

    A(int id) {
        this->id = id;
    }

    A(const A &a) {
        this->id = a.id;
    }

    A& operator = ( const A &other ) {
        if ( this == &other ) {
            return *this;
        }

        this->id = other.id;
    }

    ~A() {
        std::cout << id << " gets destoried. " << std::endl;
    }

    friend std::ostream& operator << ( std::ostream &out, const A& a ) {
        out << "id: " << a.id;
    }
public:
    int id;
};

int main( int argc, char** argv ) {
    std::cout << "Hello, TrySmartPointer! " << std::endl;

    A a0(0);

    std::cout << "a0: " << a0 << std::endl;

    A a0_1(a0);

    std::cout << "a0_1: " << a0_1 << std::endl;

    A a0_2;

    a0_2 = a0_1;

    std::cout << "a0_2: " << a0_2 << std::endl;

    auto sa0 = std::make_shared<A>( a0 );

    std::cout << "sa0: " << *sa0 << std::endl;

    sa0->id = 1;

    std::cout << "sa0: " << *sa0 << std::endl;
    std::cout << "a0: " << a0 << std::endl;

    sa0.reset();

    std::cout << "sa0.get() = " << sa0.get() << std::endl;

    if ( nullptr == sa0.get() ) {
        std::cout << "sa0.get() is nullptr. " << std::endl;
    } else {
        std::cout << "sa0.get() is not nullptr. " << std::endl;
    }

    std::shared_ptr<A> sa2 = std::make_shared<A>( 2 );
    std::cout << "sa2: " << *sa2 << std::endl;

    std::shared_ptr<A> sa3 = std::make_shared<A>( 3 );

    sa2 = sa3;

    std::cout << "sa2: " << *sa2 << std::endl;

    std::shared_ptr<A> sa4(nullptr);

    std::cout << "sa4.get() = " << sa4.get() << std::endl;

    sa4.reset();

    std::cout << "After as4.reset(). " << std::endl;

    return 0;
}