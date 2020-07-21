//
// Created by yaoyu on 7/20/20.
//

#include <iostream>

#include "Args/ArgsParser.hpp"

static ap::Args handle_args( int argc, char** argv ) {
    ap::Args args;

    args.add_positional<std::string>("pos-1", "Positional 1. ");
    args.add_positional<float>("pos-2", "Positional 2. ");
    args.add_required<std::string>("required", "Required argument. ");
    args.add_default<int>("default", "Default argument. ", 1);
    args.add_flag("flag-1", "Flag argument. ");
    args.add_flag("flag-2", "Flag argument. ");

    ap::ValidatorFunction_t vf = []()->bool{ return ap::Args::arguments<double>["validated"]->get() > 0; };
    args.add_required<double>("validated", "Validated argument. ", vf);

    args.parse_args( argc, argv );

    std::cout << args;

    return args;
}

int main( int argc, char** argv ) {
    std::cout << "Hello, TryArgumentParser! \n";
    ap::Args args = handle_args( argc, argv );

    return 0;
}