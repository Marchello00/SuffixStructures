#ifndef REFRAINSOLVER_SUFFIX_AUTOMATON_H
#define REFRAINSOLVER_SUFFIX_AUTOMATON_H

#include <iostream>
#include <vector>
#include <numeric>
#include <stack>
#include <cassert>
#include <memory>
#include "node.h"
#include "container.h"

template<typename Container = VectorNodesContainer<SuffAutomatonNode<MapNode<>>>>
class SuffixAutomaton : public Container {
    typedef int IndexT;
    typedef typename Container::Base Base;
    typedef typename Container::NodeType NodeType;

    class Iterator;

public:
    typedef typename Base::SymbolT SymbolT;

    typedef Iterator iterator;

    SuffixAutomaton() {
        lastNode_ = root_ = Container::newVert(0);
    }

    void addLetter(SymbolT c) {
        IndexT parent = lastNode_;
        lastNode_ = Container::newVert(Container::get(parent).len() + 1);
        for (; !Container::get(parent).have(c); parent = Container::get(parent).suff()) {
            Container::get(parent).add(c, lastNode_);
        }
        if (Container::get(parent).get(c) == lastNode_) {
            Container::get(lastNode_).setSufflink(0);
            return;
        }
        IndexT candidate = Container::get(parent).get(c);
        if (Container::get(candidate).len() == Container::get(parent).len() + 1) {
            Container::get(lastNode_).setSufflink(candidate);
            return;
        }
        IndexT clone = Container::copyNode(candidate);
        Container::get(clone).setLen(Container::get(parent).len() + 1);
        Container::get(lastNode_).setSufflink(clone);
        Container::get(candidate).setSufflink(clone);
        for (; Container::get(parent).get(c) == candidate; parent = Container::get(parent).suff()) {
            Container::get(parent).add(c, clone);
        }
    }

    void add(const std::string &s) {
        for (auto c : s) {
            addLetter(c);
        }
    }

    void operator+=(const std::string &s) {
        add(s);
    }

    Iterator root() const {
        return Iterator(this, root_);
    }

    void fillTerminals() {
        fillTerminals_(lastNode_);
    }

private:
    IndexT lastNode_;
    IndexT root_;

    void fillTerminals_(int current) {
        if (!current) return;
        Container::get(current).setTerminal();
        fillTerminals_(Container::get(current).suff());
    }

    class Iterator : public Container::template iterator<Iterator> {
        typedef SuffixAutomaton<Container> MyFather;
        friend MyFather;
        typedef typename Container::template iterator<Iterator> CIter;
    public:
        typedef typename Base::SymbolT SymbolT;

        IndexT suff() const {
            return CIter::father_->cget(CIter::ind_).suff();
        }

        bool isTerminal() const {
            return CIter::father_->cget(CIter::ind_).isTerminal();
        }

        IndexT len() const {
            return CIter::father_->cget(CIter::ind_).len();
        }

        explicit Iterator(const typename CIter::Father *father, IndexT ind) :
                CIter(father, ind) {}
    };
};

#endif //REFRAINSOLVER_SUFFIX_AUTOMATON_H
