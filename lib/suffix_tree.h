#ifndef REFRAINSOLVER_SUFFIX_TREE_H
#define REFRAINSOLVER_SUFFIX_TREE_H

#include <iostream>
#include <vector>
#include <numeric>
#include <stack>
#include <cassert>
#include <memory>
#include "node.h"
#include "container.h"

template<typename Container = VectorNodesContainer<SuffTreeNode<MapNode<>>>>
class SuffixTree : public Container {
public:
    typedef typename Container::Base Base;
    typedef typename Container::NodeType NodeType;
private:
    class Iterator;

    typedef int IndexT;
public:
    typedef typename Base::SymbolT SymbolT;

    typedef Iterator iterator;

    explicit SuffixTree(const std::string &s = "") {
        fictive_ = Container::newVert(-1, -1);
        root_ = Container::newVert(-1, 0);
        Container::get(root_).setSufflink(fictive_);
        Container::get(root_).setParent(fictive_);
        current_ = root_;
        pos_ = 0;
        Container::get(fictive_).setFictive(root_);
        add(s);
    }

    void addLetter(char letter) {
        str_ += letter;
        while (true) {
            if (pos_ == Container::get(current_).r()) {
                if (!Container::get(current_).have(letter)) {
                    newLeaf_(current_);
                    current_ = Container::get(current_).suff();
                    pos_ = Container::get(current_).r();
                } else {
                    current_ = Container::get(current_).get(letter);
                    pos_ = Container::get(current_).l() + 1;
                    return;
                }
            } else {
                if (str_[pos_] == letter) {
                    ++pos_;
                    return;
                } else {
                    int u = split_(current_, pos_);
                    newLeaf_(u);
                    move_(Container::get(Container::get(u).parent()).suff(),
                          Container::get(u).l(), Container::get(u).r());
                    if (pos_ == Container::get(current_).r()) {
                        Container::get(u).setSufflink(current_);
                    } else {
                        Container::get(u).setSufflink(Container::size());
                    }
                }
            }
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

    bool find(const std::string &subs) {
        int subsPos = 0;
        int treeV = root_;
        for (;;) {
            if (!Container::get(treeV).have(subs[subsPos])) {
                return false;
            }
            treeV = Container::get(treeV).get(subs[subsPos]);
            int j = Container::get(treeV).l();
            for (; j < Container::get(treeV).r() && subsPos < subs.size() && str_[j] == subs[subsPos];
                   ++j, ++subsPos) {}
            if (subsPos < subs.size() && j < Container::get(treeV).r()) {
                return false;
            }
            if (subsPos == subs.size()) {
                return true;
            }
        }
    }

    Iterator root() const {
        return Iterator(this, root_);
    }

private:
    const IndexT INF = (IndexT) 1e9; // max string length

    IndexT fictive_ = 0, root_ = 1;

    std::string str_;

    IndexT current_, pos_;

    void connect_(int parent, int son) {
        Container::get(parent).add(str_[Container::get(son).l()], son);
        Container::get(son).setParent(parent);
    }

    // parent --> nvert --> son
    IndexT split_(IndexT son, IndexT pos) {
        IndexT parent = Container::get(son).parent();
        IndexT nvert = Container::newVert(Container::get(son).l(), pos);
        Container::get(son).setL(Container::get(nvert).r());
        connect_(parent, nvert);
        connect_(nvert, son);
        return nvert;
    }

    IndexT newLeaf_(IndexT parent) {
        IndexT leaf = Container::newVert(str_.size() - 1, INF);
        connect_(parent, leaf);
        return leaf;
    }

    void move_(IndexT from, IndexT l, IndexT r) {
        if (l == r) {
            current_ = from;
            pos_ = Container::get(current_).r();
            return;
        }
        from = Container::get(from).get(str_[l]);
        while (Container::get(from).r() - Container::get(from).l() < r - l) {
            l += Container::get(from).r() - Container::get(from).l();
            from = Container::get(from).get(str_[l]);
        }
        current_ = from;
        pos_ = Container::get(current_).l() + r - l;
    }

    class Iterator : public Container::template iterator<Iterator> {
        typedef SuffixTree<Container> MyFather;
        friend MyFather;
        typedef typename Container::template iterator<Iterator> CIter;
    public:
        IndexT l() const {
            return CIter::father_->cget(CIter::ind_).l();
        }

        IndexT r() const {
            return CIter::father_->cget(CIter::ind_).r();
        }

        IndexT suff() const {
            return CIter::father_->cget(CIter::ind_).suff();
        }

        bool isLeaf() const {
            return !(CIter::begin() != CIter::end());
        }

        IndexT len() const {
            return std::min(r(), (IndexT) dynamic_cast<const MyFather *>(CIter::father_)->str_.size() - 1) -
                   std::max(l(), 0);
        }

        explicit Iterator(const typename CIter::Father *father, IndexT ind) :
                CIter(father, ind) {}
    };
};

#endif //REFRAINSOLVER_SUFFIX_TREE_H
