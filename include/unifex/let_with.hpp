/*
 * Copyright 2019-present Facebook, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <unifex/config.hpp>
#include <unifex/just.hpp>
#include <unifex/let.hpp>
#include <unifex/receiver_concepts.hpp>
#include <unifex/sender_concepts.hpp>
#include <unifex/execution_policy.hpp>
#include <unifex/type_list.hpp>

#include <unifex/detail/prologue.hpp>

namespace unifex {

namespace _let_with {

template <typename InnerOp, typename Receiver, typename... StateFactories>
struct _operation {
  struct type;
};

template <typename InnerOp, typename Receiver, typename... StateFactories>
using operation =  typename _operation<
    InnerOp, Receiver, StateFactories...>::type;

template<typename SuccessorFactory, typename... StateFactory>
struct _sender {
    class type;
};

template<typename SuccessorFactory, typename... StateFactories>
using let_with_sender = typename _sender<SuccessorFactory, StateFactories...>::type;

template<typename SuccessorFactory, typename... StateFactories>
class _sender<SuccessorFactory, StateFactories...>::type {
public:
    using InnerOp = std::invoke_result_t<SuccessorFactory, std::callable_result_t<StateFactories>&...>;

    template<template<typename...> class Variant, template<typename...> class Tuple>
    using value_types = typename InnerOp::template value_types<Variant, Tuple>;

    template<template<typename...> class Variant>
    using error_types = typename InnerOp::template error_types<Variant>;

    template<typename StateFactoryTuple, typename SuccessorFactory2>
    explicit type(StateFactoryTuple&& stateFactoryTuple, SuccessorFactory2&& func) :
        stateFactories_((StateFactoryTuple&&)stateFactoryTuple),
        func_((SuccessorFactory2&&)func)
    {}

    template(typename Self, typename Receiver)
        (requires same_as<remove_cvref_t<Self>, type> AND receiver<Receiver>)
    friend auto tag_invoke(tag_t<unifex::connect>, Self&& self, Receiver&& r)
        noexcept(
            (std::is_nothrow_callable_v<member_t<Self, StateFactories>> && ... ) /*&&
            std::is_nothrow_invocable_v<
                member_t<Self, SuccessorFactory>,
                std::invoke_result_t<member_t<Self, StateFactories>>&>,... &&
            is_nothrow_connectable_v<
                callable_result_t<
                    member_t<Self, SuccessorFactory>,
                    std::invoke_result_t<member_t<Self, StateFactories>>&, ...>,
                remove_cvref_t<Receiver>>*/) {

        return operation<
                member_t<Self, SuccessorFactory>, Receiver, member_t<Self, StateFactories>...>(
            static_cast<Self&&>(self).stateFactories_,
            static_cast<Self&&>(self).func_,
            static_cast<Receiver&&>(r));
    }

private:
    UNIFEX_NO_UNIQUE_ADDRESS std::tuple<StateFactories...> stateFactories_;
    UNIFEX_NO_UNIQUE_ADDRESS SuccessorFactory func_;
};

// Conversion helper to support in-place construction via RVO
template<typename Target, typename Func>
struct Converter {
    operator Target() {
        return std::move(func_)();
    }
    Func func_;
};

template<typename SuccessorFactory, typename Receiver, typename... StateFactories>
struct _operation<SuccessorFactory, Receiver, StateFactories...>::type {
    using StateTupleT = std::tuple<std::invoke_result_t<StateFactories>...>;
    type(std::tuple<StateFactories...>&& stateFactories, SuccessorFactory&& func, Receiver&& r) :
        stateFactory_(static_cast<StateFactory&&>(stateFactory)),
        func_(static_cast<SuccessorFactory&&>(func)),
        // Construct the tuple of state from the tuple of factories
        // using in-place construction via RVO
        state_(std::apply([](auto&&... stateFactory){
            return StateTupleT(
                Converter<std::invoke_result_t<std::remove_cvref_t<decltype(stateFactory)>&&>, std::remove_cvref_t<decltype(stateFactory)>>{
                    std::move(stateFactory)}...
            );
        },
        std::move(stateFactories))),
        innerOp_(
              unifex::connect(
                std::apply(static_cast<SuccessorFactory&&>(func), state_),
                static_cast<Receiver&&>(r))) {
    }

    void start() & noexcept {
        unifex::start(innerOp_);
    }

    StateFactory stateFactory_;
    SuccessorFactory func_;
    StateTupleT state_;

    connect_result_t<
        callable_result_t<SuccessorFactory&&, callable_result_t<StateFactories>&...>,
        remove_cvref_t<Receiver>>
        innerOp_;
};

struct _fn {
#if 0
    template<typename StateFactory, typename SuccessorFactory>
    auto operator()(StateFactory&& stateFactory, SuccessorFactory&& successor_factory) const
        noexcept(std::is_nothrow_constructible_v<remove_cvref_t<SuccessorFactory>, SuccessorFactory> &&
                 std::is_nothrow_constructible_v<remove_cvref_t<StateFactory>, StateFactory>)
        -> let_with_sender<remove_cvref_t<StateFactory>, remove_cvref_t<SuccessorFactory>> {
        return let_with_sender<remove_cvref_t<StateFactory>, remove_cvref_t<SuccessorFactory>>{
            (StateFactory&&)stateFactory, (SuccessorFactory&&)successor_factory};
    }
#endif

    template<typename SuccessorFactory, typename... StateFactories>
    auto let_with_builder(std::tuple<StateFactories...>&& stateFactories, SuccessorFactory&& successorFactory) const {
        return let_with_sender<remove_cvref_t<SuccessorFactory>, remove_cvref_t<StateFactories>...>{
            (std::tuple<StateFactories...>&&)stateFactories, (SuccessorFactory&&)successorFactory};
    }

    template<typename StateFactory, typename SuccessorFactory>
    auto let_with_helper(StateFactory&& stateFactory, SuccessorFactory&& successorFactory) const {
        return std::pair<std::tuple<StateFactory>, SuccessorFactory>(
            std::tuple<StateFactory>{(StateFactory&&)stateFactory},
            (SuccessorFactory&&)successorFactory);
    }

    template<typename Factory, typename... Factories>
    auto let_with_helper(Factory&& factory, Factories&&... factories) const {
        auto p = let_with_helper((Factories&&)factories...);
        return make_pair(
            std::tuple_cat(
                std::make_tuple((Factory&&)factory),
                std::move(p.first)),
            std::move(p.second)
        );
    }

public:
    template<typename... Factories>
    auto operator()(Factories&&... factories) const {
        auto p = let_with_helper((Factories&&)factories...);
        return let_with_builder(std::move(p.first), std::move(p.second));
    }
};

} // namespace _let_with

namespace _let_with_cpo {
    struct _fn {

        template<typename StateFactory, typename SuccessorFactory>
        auto operator()(StateFactory&& stateFactory, SuccessorFactory&& successor_factory) const
            noexcept(std::is_nothrow_constructible_v<std::decay_t<SuccessorFactory>, SuccessorFactory> &&
                    std::is_nothrow_constructible_v<std::decay_t<StateFactory>, StateFactory>)
            -> _let_with::let_with_sender<
                 std::decay_t<StateFactory>, std::decay_t<SuccessorFactory>> {
            return _let_with::let_with_sender<
                std::decay_t<StateFactory>, std::decay_t<SuccessorFactory>>{
                (StateFactory&&)stateFactory, (SuccessorFactory&&)successor_factory};
        }
    };
} // namespace _let_with_cpo

inline constexpr _let_with_cpo::_fn let_with{};


} // namespace unifex

#include <unifex/detail/epilogue.hpp>
