//
// Created by yaoyu on 7/20/20.
//

#ifndef INCLUDES_ARGS_ARGSPARSER_HPP
#define INCLUDES_ARGS_ARGSPARSER_HPP

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <map>
#include <vector>

#include "Args.hpp"

// Namespaces.
namespace bpo = boost::program_options;

namespace ap {

typedef std::function<bool()> ValidatorFunction_t;

struct DefaultValidator {
    bool operator()() {
        return true;
    }
};

template<typename ValueT>
class SingleArgument {
public:
    SingleArgument(const std::string &name, const std::string &desc,
                   ValidatorFunction_t validator = DefaultValidator())
            : name{name}, description{desc}, validator{validator} {
        show = [this]() -> std::string { return this->str(); };
    }

    virtual ~SingleArgument() = default;

    virtual void add_option(
            bpo::options_description &optDesc,
            bpo::positional_options_description &optPosDesc) = 0;

    virtual const ValueT get() const {
        return value;
    }

    virtual bool validate() {
        return validator();
    }

    virtual std::string str() const {
        std::stringstream ss;
        ss << this->name << ": " << this->value;
        return ss.str();
    }

public:
    std::string name;
    std::string description;
    ValueT value;
    ValidatorFunction_t validator;
    std::function<std::string()> show;
};

template<typename ValueT>
class RequiredArgument : public SingleArgument<ValueT> {
public:
    RequiredArgument(
            const std::string &name,
            const std::string &desc,
            ValidatorFunction_t validator = DefaultValidator())
            : SingleArgument<ValueT>(name, desc, validator) {}

    virtual ~RequiredArgument() = default;

    void add_option(
            bpo::options_description &optDesc,
            bpo::positional_options_description &optPosDesc) override {
        optDesc.add_options()(
                this->name.c_str(),
                bpo::value<ValueT>(&(this->value))->required(),
                this->description.c_str());
    }
};

template<typename ValueT>
class DefaultArgument : public SingleArgument<ValueT> {
public:
    DefaultArgument(
            const std::string &name,
            const std::string &desc,
            const ValueT &defaultValue,
            ValidatorFunction_t validator = DefaultValidator())
            : SingleArgument<ValueT>(name, desc, validator),
              defaultValue{defaultValue} {}

    virtual ~DefaultArgument() = default;

    void add_option(
            bpo::options_description &optDesc,
            bpo::positional_options_description &optPosDesc) override {
        optDesc.add_options()(
                this->name.c_str(),
                bpo::value<ValueT>(&(this->value))->default_value(this->defaultValue),
                this->description.c_str());
    }

public:
    ValueT defaultValue;
};

template<typename ValueT>
class PositionalArgument : public SingleArgument<ValueT> {
public:
    PositionalArgument(
            const std::string &name,
            const std::string &desc,
            ValidatorFunction_t validator = DefaultValidator())
            : SingleArgument<ValueT>(name, desc, validator) {}

    virtual ~PositionalArgument() = default;

    void add_option(
            bpo::options_description &optDesc,
            bpo::positional_options_description &optPosDesc) override {
        optDesc.add_options()(
                this->name.c_str(),
                bpo::value<ValueT>(&(this->value))->required(),
                this->description.c_str());

        optPosDesc.add(this->name.c_str(), 1);
    }
};

class FlagArgument : public SingleArgument<bool> {
public:
    typedef bool FlagT;
public:
    FlagArgument(
            const std::string &name,
            const std::string &desc,
            ValidatorFunction_t validator = DefaultValidator())
            : SingleArgument<bool>(name, desc, validator), tempFlagValue{0} {}

    virtual ~FlagArgument() = default;

    void add_option(
            bpo::options_description &optDesc,
            bpo::positional_options_description &optPosDesc) override {

        optDesc.add_options()(
                this->name.c_str(),
                bpo::value<int>(&(this->tempFlagValue))->implicit_value(1)->default_value(0),
                this->description.c_str());
    }

    const bool get() const override {
        return (0 != tempFlagValue);
    }

    std::string str() const override {
        std::stringstream ss;
        if (this->get()) {
            ss << this->name << ": true";
        } else {
            ss << this->name << ": false";
        }

        return ss.str();
    }

public:
    int tempFlagValue;
};

class Args {
public:
    Args() : pOptDesc{std::make_unique<bpo::options_description>("Dummy description")},
             pPosOptDesc{std::make_unique<bpo::positional_options_description>()} {}

    ~Args() = default;

    bool validate() const {
        bool flag = true;

        for (int i = 0; i < validatorFunctions.size(); ++i) {
            bool f = validatorFunctions[i]();

            if (!f) {
                std::cerr << "Argument \"" << names[i] << "\" validation failed. \n";
            }

            flag = flag && f;
        }

        return flag;
    }

    friend std::ostream &operator<<(std::ostream &out, const Args &args) {
        for (auto &show : args.showFunctions) {
            out << show() << "\n";
        }

        return out;
    }

    template<typename T>
    void add_required(
            const std::string &name,
            const std::string &desc,
            ValidatorFunction_t validator = DefaultValidator());

    template<typename T>
    void add_default(
            const std::string &name,
            const std::string &desc,
            const T &defaultValue,
            ValidatorFunction_t validator = DefaultValidator());

    template<typename T>
    void add_positional(
            const std::string &name,
            const std::string &desc,
            ValidatorFunction_t validator = DefaultValidator());

    void add_flag(
            const std::string &name,
            const std::string &desc,
            ValidatorFunction_t validator = DefaultValidator());

    void parse_args(int argc, char *argv[]) {
        try {
            bpo::variables_map optVM;
            bpo::store(bpo::command_line_parser(argc, argv).
                    options(*pOptDesc).positional(*pPosOptDesc).run(), optVM);
            bpo::notify(optVM);
        }
        catch (std::exception &e) {
            std::cout << e.what() << std::endl;
            throw (e);
        }

        if (!validate()) {
            EXCEPTION_INVALID_ARGUMENTS_IN_CLASS()
        }
    }

public:
    template<typename ValueT>
    static std::map<std::string, std::unique_ptr<SingleArgument<ValueT>>> arguments;

    std::vector<std::string> names;
    std::vector<ValidatorFunction_t> validatorFunctions;
    std::vector<std::function<std::string()> > showFunctions;

    std::shared_ptr<bpo::options_description> pOptDesc;
    std::shared_ptr<bpo::positional_options_description> pPosOptDesc;
};

template<typename ValueT>
std::map<std::string, std::unique_ptr<SingleArgument<ValueT> > > Args::arguments;

template<typename T>
void Args::add_required(
        const std::string &name,
        const std::string &desc,
        ValidatorFunction_t validator) {
    auto r = std::make_unique<RequiredArgument<T>>(name, desc, validator);
    r->add_option(*pOptDesc, *pPosOptDesc);
    names.push_back(name);
    validatorFunctions.push_back(r->validator);
    showFunctions.push_back(r->show);
    Args::arguments<T>[name] = std::move(r);
}

template<typename T>
void Args::add_default(
        const std::string &name,
        const std::string &desc,
        const T &defaultValue,
        ValidatorFunction_t validator) {
    auto d = std::make_unique<DefaultArgument<T>>(name, desc, defaultValue, validator);
    d->add_option(*pOptDesc, *pPosOptDesc);
    names.push_back(name);
    validatorFunctions.push_back(d->validator);
    showFunctions.push_back(d->show);
    Args::arguments<T>[name] = std::move(d);
}

template<typename T>
void Args::add_positional(
        const std::string &name,
        const std::string &desc,
        ValidatorFunction_t validator) {
    auto p = std::make_unique<PositionalArgument<T>>(name, desc, validator);
    p->add_option(*pOptDesc, *pPosOptDesc);
    names.push_back(name);
    validatorFunctions.push_back(p->validator);
    showFunctions.push_back(p->show);
    Args::arguments<T>[name] = std::move(p);
}

void Args::add_flag(
        const std::string &name,
        const std::string &desc,
        ValidatorFunction_t validator) {
    auto f = std::make_unique<FlagArgument>(name, desc, validator);
    f->add_option(*pOptDesc, *pPosOptDesc);
    names.push_back(name);
    validatorFunctions.push_back(f->validator);
    showFunctions.push_back(f->show);
    Args::arguments<FlagArgument::FlagT>[name] = std::move(f);
}

} // namespace ap
#endif //INCLUDES_ARGS_ARGSPARSER_HPP
