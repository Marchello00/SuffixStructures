#ifndef REFRAINSOLVER_CONTAINER_H
#define REFRAINSOLVER_CONTAINER_H

#include <iostream>
#include <vector>
#include <numeric>
#include <stack>
#include <cassert>
#include <memory>
#include "node.h"

template<typename StoreValue_, typename ContIndexT_>
class ContainerI {
public:
    typedef ContIndexT_ ContIndexT;
    typedef StoreValue_ StoreValue;

//    template<typename... Args>
//    IndexT newVert(Args... args) {}

    virtual StoreValue &get(ContIndexT index) = 0;

    virtual const StoreValue &cget(ContIndexT index) const = 0;
};

template<typename NodeType_>
class VectorNodesContainer : public ContainerI<NodeType_, int> {
public:
    typedef NodeType_ NodeType;
    typedef typename NodeType::Base Base;
private:
    template<typename RealIterator>
    class Iterator;

public:
    template<typename RealIterator>
    using iterator = Iterator<RealIterator>;

    typedef int IndexT;

    typedef typename Base::SymbolT SymbolT;
    typedef IndexT ContainerIndex;

    template<typename... Args>
    IndexT newVert(Args... args) {
        nodes_.emplace_back(args...);
        return nodes_.size() - 1;
    }

    IndexT copyNode(int ind) {
        nodes_.emplace_back(nodes_[ind]);
        return nodes_.size() - 1;
    }

    NodeType &get(IndexT index) override {
        return nodes_[index];
    }

    const NodeType &cget(IndexT index) const override {
        return nodes_[index];
    };

    IndexT size() const {
        return nodes_.size();
    }

private:

    std::vector<NodeType> nodes_;

    template<typename RealIterator>
    class Iterator : public IteratorArrayIndex<VectorNodesContainer<NodeType>, SubIteratorArrayIndex<RealIterator>> {
        protected:
        friend class VectorNodesContainer<NodeType>;

        friend class SubIteratorArrayIndex<Iterator>;

        typedef IteratorArrayIndex<VectorNodesContainer<NodeType>, SubIteratorArrayIndex<RealIterator>> BIter;
        typedef VectorNodesContainer<NodeType> Father;

        public:
        typedef typename Base::SymbolT SymbolT;

        protected:
        explicit Iterator(const Father *father, IndexT ind) :
                BIter(father, ind) {}
    };
};

#endif //REFRAINSOLVER_CONTAINER_H
