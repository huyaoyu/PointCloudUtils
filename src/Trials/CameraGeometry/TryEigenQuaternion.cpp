//
// Created by yaoyu on 4/1/20.
//

#include <iostream>

#include <Eigen/Dense>

int main(int argc, char* argv[]) {
    std::cout << "Hello, TryEigenQuaternion. " << std::endl;

    Eigen::Vector4f qv;
    // x, y, z, w.
    qv << 0.250626, -0.0200405, -0.0184595, 0.967701;

    Eigen::Quaternionf q(qv);

    std::cout << "qv: " << std::endl << qv << std::endl;
    std::cout << "q.w() = " << q.w() << std::endl;
    std::cout << "q.x() = " << q.x() << std::endl;
    std::cout << "q.y() = " << q.y() << std::endl;
    std::cout << "q.z() = " << q.z() << std::endl;
    std::cout << "q.toRotationMatrix:" << std::endl << q.toRotationMatrix() << std::endl;

    return 0;
}