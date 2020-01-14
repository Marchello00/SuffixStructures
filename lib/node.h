#ifndef REFRAINSOLVER_NODE_H
#define REFRAINSOLVER_NODE_H

#include <iostream>
#include <vector>
#include <map>
#include <algorithm>

template<typename LinkT = int, typename Creator = LinkT>
class BasicNode {
protected:
    typedef int IndexT;
    typedef long long CountT;
    typedef LinkT LinkT_;

public:
    typedef int SymbolT;

    virtual LinkT get(SymbolT symbol) const = 0;

    virtual bool have(SymbolT symbol) const = 0;

    virtual void add(SymbolT symbol, const LinkT &) = 0;

    virtual void set(SymbolT symbol, const LinkT &) = 0;

    template<typename... Args>
    void add(SymbolT symbol, Args... args) {
        add(symbol, Creator(args...));
    }

    template<typename T>
    static constexpr T fictive() noexcept {
        return T();
    }

    static constexpr int fictive() noexcept {
        return -1;
    }
};

template<typename LinkT = int, typename Creator = LinkT>
class MapNode : public BasicNode<LinkT, Creator> {
protected:
    typedef BasicNode<LinkT, Creator> Father;

public:
    typedef typename Father::SymbolT SymbolT;
protected:
    typedef typename Father::IndexT IndexT;
    typedef typename Father::CountT CountT;

    typedef std::map<SymbolT, LinkT> Container;

public:
    typedef MapNode<LinkT, Creator> Base;
    static constexpr LinkT fictive = Father::template fictive<LinkT>();
    typedef typename Container::const_iterator iterator;

    LinkT get(SymbolT symbol) const override {
        auto ptr = next_.find(symbol);
        return (ptr == next_.end() ? fictive : ptr->second);
    }

    bool have(SymbolT symbol) const override {
        auto ptr = next_.find(symbol);
        return ptr != next_.end();
    }

    void add(SymbolT symbol, const LinkT &to) override {
        next_[symbol] = to;
    }

    void set(SymbolT symbol, const LinkT &to) override {
        next_[symbol] = to;
    }

    typename Container::const_iterator begin() const {
        return next_.cbegin();
    }

    typename Container::const_iterator end() const {
        return next_.cend();
    }

private:
    Container next_;
};

template<unsigned ALPH_LEN = 'z' - 'a' + 1, unsigned ALPH_START = 'a',
        typename LinkT = int, typename Creator = LinkT>
class ArrayNode : public BasicNode<LinkT, Creator> {
    class Iterator;

    friend class Iterator;

protected:
    typedef BasicNode<LinkT, Creator> Father;

public:
    typedef typename Father::SymbolT SymbolT;
protected:
    typedef typename Father::IndexT IndexT;
    typedef typename Father::CountT CountT;

public:
    typedef ArrayNode<ALPH_LEN, ALPH_START, LinkT, Creator> Base;
    typedef Iterator iterator;

    ArrayNode() {
        std::fill_n(next_, ALPH_LEN, fictive);
    }

    static constexpr LinkT fictive = Father::template fictive<LinkT>();

    LinkT get(SymbolT symbol) const override {
        return next_[getIndex_(symbol)];
    }

    bool have(SymbolT symbol) const override {
        return next_[getIndex_(symbol)] != fictive;
    }

    void add(SymbolT symbol, const LinkT &to) override {
        next_[getIndex_(symbol)] = to;
    }

    void set(SymbolT symbol, const LinkT &to) override {
        next_[getIndex_(symbol)] = to;
    }

    Iterator begin() const {
        Iterator iter(this, -1);
        ++iter;
        return iter;
    }

    Iterator end() const {
        return Iterator(this, ALPH_LEN);
    }

protected:
    IndexT getIndex_(SymbolT symbol) const {
        if (symbol >= ALPH_START + ALPH_LEN || symbol < ALPH_START) {
            throw std::overflow_error("Some of ALPH_START = " + std::to_string(ALPH_START) +
                                      ", ALPH_LEN = " + std::to_string(ALPH_LEN) +
                                      " or symbol = " + std::to_string(symbol) + " are incorrect");
        }
        return symbol - ALPH_START;
    }

private:
    LinkT next_[ALPH_LEN]{};

    class Iterator : public std::iterator<std::input_iterator_tag, std::pair<SymbolT, LinkT>> {
        friend class ArrayNode<ALPH_LEN, ALPH_START>;

    public:
        bool operator!=(const Iterator &iterator) const {
            return father_ != iterator.father_ || index_ != iterator.index_;
        }

        std::pair<SymbolT, LinkT> operator*() const {
            return std::make_pair(index_ + ALPH_START, father_->next_[index_]);
        }

        void operator++() {
            for (++index_; index_ < ALPH_LEN; ++index_) {
                if (father_->next_[index_] != fictive) {
                    break;
                }
            }
        }

    private:
        Iterator(const ArrayNode<ALPH_LEN, ALPH_START> *father, IndexT index) :
                index_(index), father_(father) {}

        IndexT index_;
        const ArrayNode<ALPH_LEN, ALPH_START> *father_;

    };
};

template<typename Father>
class SuffStructNode : public virtual Father {
protected:
    typedef typename Father::SymbolT SymbolT;
    typedef typename Father::IndexT IndexT;
    typedef typename Father::CountT CountT;

    typedef typename Father::LinkT_ LinkT;
public:
    template<typename... Args>
    explicit SuffStructNode(Args ...args) :
            Father(args...) {}

    void setSufflink(LinkT sufflink) {
        sufflink_ = sufflink;
    }

    LinkT suff() const {
        return sufflink_;
    }

private:
    LinkT sufflink_ = 0;
};

template<typename Father>
class TypedNode : public virtual Father {
protected:
    typedef typename Father::SymbolT SymbolT;
    typedef typename Father::IndexT IndexT;
    typedef typename Father::CountT CountT;

    typedef typename Father::LinkT_ LinkT;
public:
    enum {
        BASIC = 0b0u,
        FICTIVE = 0b1u,
        TERMINAL = 0b10u
    };

    template<typename... Args>
    explicit TypedNode(Args ...args) :
            Father(args...) {}

    void setFictive(LinkT root) {
        type_ |= FICTIVE;
        root_ = root;
    }

    bool isFictive() const {
        return type_ & FICTIVE;
    }

    void setTerminal() {
        type_ |= TERMINAL;
    }

    bool isTerminal() const {
        return type_ & TERMINAL;
    }

    LinkT get(SymbolT symbol) const override {
        if (isFictive()) {
            return root_;
        }
        return Father::get(symbol);
    }

    bool have(SymbolT symbol) const override {
        if (isFictive()) {
            return root_;
        }
        return Father::have(symbol);
    }

private:
    unsigned type_ = BASIC;
    LinkT root_ = Father::fictive;
};

template<typename Base>
class SuffTreeNode : public SuffStructNode<Base>, public TypedNode<Base> {
    typedef SuffStructNode<Base> Father;

    typedef typename Father::SymbolT SymbolT;
    typedef typename Father::IndexT IndexT;
    typedef typename Father::CountT CountT;

    typedef typename Base::LinkT_ LinkT;
public:
    template<typename... Args>
    SuffTreeNode(IndexT l, IndexT r, Args ...args) :
            l_(l), r_(r), Father(args...) {}

    IndexT l() const {
        return l_;
    }

    IndexT r() const {
        return r_;
    }

    IndexT parent() const {
        return parent_;
    }

    void setL(IndexT l) {
        l_ = l;
    }

    void setR(IndexT r) {
        r_ = r;
    }

    void setParent(LinkT parent) {
        parent_ = parent;
    }

private:
    LinkT parent_ = 0;
    IndexT l_ = 0, r_ = 0;
};

template<typename Base>
class SuffAutomatonNode : public SuffStructNode<Base>, public TypedNode<Base> {
    typedef SuffStructNode<Base> Father;

    typedef typename Father::SymbolT SymbolT;
    typedef typename Father::IndexT IndexT;
    typedef typename Father::CountT CountT;

    typedef typename Base::LinkT_ LinkT;
public:
    template<typename... Args>
    explicit SuffAutomatonNode(IndexT len, Args ...args) :
            len_(len), Father(args...) {}

    IndexT len() const {
        return len_;
    }

    void setLen(IndexT len) {
        len_ = len;
    }

private:
    IndexT len_ = 0;
};

template<typename IterBase>
class SubIteratorArrayIndex : public std::iterator<std::input_iterator_tag,
        std::pair<typename IterBase::SymbolT, IterBase>> {
    typedef typename IterBase::Father_ Father;

public:
    bool operator!=(const SubIteratorArrayIndex &iterator) const {
        return father_ != iterator.father_ || base_ != iterator.base_;
    }

    std::pair<typename IterBase::SymbolT, IterBase> operator*() const {
        auto p = *base_;
        return std::make_pair(p.first, IterBase(father_, p.second));
    }

    void operator++() {
        ++base_;
    }

    explicit SubIteratorArrayIndex(const Father *father, const typename Father::Base::iterator &base) :
            father_(father), base_(base) {}

private:

    typename Father::Base::iterator base_;
    const Father *father_;
};

template<typename Father, typename SubIterator>
class IteratorArrayIndex {
public:
    typedef typename Father::SymbolT SymbolT;
    typedef typename Father::IndexT IndexT;
    typedef Father Father_;

    bool have(SymbolT symbol) const {
        return father_->nodes_[ind_].have(symbol);
    }

    IteratorArrayIndex get(SymbolT symbol) const {
        return Iterator(father_, father_->nodes_[ind_].get(symbol));
    }

    void go(SymbolT symbol) {
        if (!have(symbol)) {
            return;
        }
        ind_ = father_->nodes_[ind_].get(symbol);
    }

    IndexT ind() const {
        return ind_;
    }

    SubIterator begin() const {
        return SubIterator(father_, father_->cget(ind_).begin());
    }

    SubIterator end() const {
        return SubIterator(father_, father_->cget(ind_).end());
    }

protected:
    explicit IteratorArrayIndex(const Father *father, IndexT ind) :
            father_(father), ind_(ind) {}

    const Father *father_;
    IndexT ind_;
};

#endif //REFRAINSOLVER_NODE_H
