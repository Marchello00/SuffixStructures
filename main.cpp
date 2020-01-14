#include <iostream>
#include <vector>
#include <numeric>
#include <stack>
#include <cassert>
#include <memory>

#ifndef STRINGS_NODE_H
#define STRINGS_NODE_H

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

#endif //STRINGS_NODE_H

template<typename T, typename Cmp = std::less<T>>
class SparseTable {
    typedef int IndexT;
    typedef unsigned LogT;
public:
    SparseTable() = default;

    explicit SparseTable(const std::vector<T> &a, const Cmp &cmp = Cmp()) :
            cmp_(cmp), n_(a.size()), log2_(n_ + 1, 0) {
        fillLog();
        f_.emplace_back(a);
        ind_.emplace_back(n_);
        std::iota(ind_[0].begin(), ind_[0].end(), 0);
        k_ = log2_[n_];
        f_.resize(k_ + 1);
        ind_.resize(k_ + 1);
        buildSparseTable_();
    }

    T min(IndexT l, IndexT r) {
        check(l, r);
        auto len = log2_[r - l];
        return std::min(f_[len][l], f_[len][r - (1U << len)], Cmp());
    }

    IndexT argmin(IndexT l, IndexT r) {
        check(l, r);
        auto len = log2_[r - l];
        if (cmp_(f_[len][l], f_[len][r - (1U << len)])) {
            return ind_[len][l];
        } else {
            return ind_[len][r - (1U << len)];
        }
    }

private:
    Cmp cmp_;
    IndexT n_{};
    LogT k_{};
    std::vector<std::vector<T>> f_;
    std::vector<std::vector<T>> ind_;
    std::vector<LogT> log2_;

    void check(IndexT l, IndexT r) {
        if (l >= r) {
            throw std::out_of_range("Param l must be less than r, but l = " + std::to_string(l) +
                                    " is greater than or equal to r = " + std::to_string(r));
        }
        if (l < 0) {
            throw std::out_of_range("Param l must be greater than or equal to 0, but l = " + std::to_string(l) +
                                    " less than 0");
        }
        if (r > n_) {
            throw std::out_of_range("Param r must be less than or equal to array size, but r = " + std::to_string(r) +
                                    "is greater than array size which is " + std::to_string(n_));
        }
    }

    void buildSparseTable_() {
        for (LogT i = 0; i < k_; ++i) {
            f_[i + 1].resize(n_ - (1U << i));
            ind_[i + 1].resize(n_ - (1U << i));
            for (IndexT j = 0; j < n_ - (1U << i); j++) {
                IndexT to = std::min<IndexT>(j + (1U << i), (IndexT) f_[i].size() - 1);
                if (cmp_(f_[i][j], f_[i][to])) {
                    f_[i + 1][j] = f_[i][j];
                    ind_[i + 1][j] = ind_[i][j];
                } else {
                    f_[i + 1][j] = f_[i][to];
                    ind_[i + 1][j] = ind_[i][to];
                }
            }
        }
    }

    void fillLog() {
        for (int i = 0, d = 1, deg = 0; i < n_ + 1; ++i) {
            if (i == d * 2) {
                d *= 2;
                deg++;
            }
            log2_[i] = deg;
        }
    }
};

class SuffixArray {
protected:
    typedef int IndexT;
public:
    explicit SuffixArray(const std::string &s) :
            s_(s), suffixArray_(s.size()),
            colour_(s.size()) {
        build_();
    }

    const std::vector<IndexT> &getSuffixArray() const {
        return suffixArray_;
    }

    const std::string &getS() const {
        return s_;
    }

private:
    struct Prefix {
        int f, s;
        int num;

        bool operator!=(const Prefix &pref) const {
            return f != pref.f || s != pref.s;
        }
    };

    void build_() {
        init_();
        int n = s_.size();
        for (int l = 0; l < n; (l ? l *= 2 : l += 1)) {
            std::vector<Prefix> prefs(n);
            for (int i = 0; i < n; ++i) {
                prefs[i] = {colour_[(suffixArray_[i] - l + n) % n], colour_[suffixArray_[i]],
                            (int) ((suffixArray_[i] - l + n) % n)};
            }
            sort_(prefs, (l ? prefs.size() : maxSym + 1));
            suffixArray_[0] = prefs[0].num;
            colour_[suffixArray_[0]] = 0;
            for (int i = 1; i < n; ++i) {
                suffixArray_[i] = prefs[i].num;
                colour_[suffixArray_[i]] = colour_[suffixArray_[i - 1]] + (prefs[i] != prefs[i - 1]);
            }
            if (colour_[suffixArray_.back()] == s_.size() - 1) {
                break;
            }
        }
    }

    void sort_(std::vector<Prefix> &a, int maxNum) {
        unsigned n = a.size();
        std::vector<int> h(maxNum, 0);
        for (auto p : a) {
            h[p.f]++;
        }
        for (int i = 1; i < maxNum; ++i) {
            h[i] += h[i - 1];
        }
        std::vector<Prefix> res(n);
        res.resize(n);
        for (int i = n - 1; i >= 0; --i) {
            res[--h[a[i].f]] = a[i];
        }
        a = res;
    }

    void init_() {
        for (int i = 0; i < s_.size(); ++i) {
            suffixArray_[i] = i;
            colour_[i] = s_[i];
            maxSym = std::max<int>(maxSym, s_[i]);
        }
    }

protected:
    std::string s_;
    std::vector<IndexT> suffixArray_;
    std::vector<IndexT> colour_;
    IndexT maxSym = 0;
};

class LCP {
    typedef int IndexT;
public:
    LCP(const std::string &s, const std::vector<IndexT> &suffixArray) :
            s_(s), suffixArray_(suffixArray),
            lcp_(s_.size()),
            pos_(s_.size()) {
        build_();
    }

    explicit LCP(const SuffixArray &suffixArray) :
            LCP(suffixArray.getS(), suffixArray.getSuffixArray()) {}

    const std::vector<IndexT> &getLcp() const {
        return lcp_;
    }

    IndexT lcp(IndexT suf1, IndexT suf2) {
        if (suf1 == suf2) {
            return s_.size() - suf1;
        }
        if (pos_[suf1] > pos_[suf2]) {
            std::swap(suf1, suf2);
        }
        return sparseTable.min(pos_[suf1], pos_[suf2] - 1);
    }

private:

    void build_() {
        fillPos_();
        IndexT currentLcp_ = 0;
        for (int i = 0; i < s_.size(); ++i) {
            if (currentLcp_ > 0) {
                --currentLcp_;
            }
            if (pos_[i] == s_.size() - 1) {
                currentLcp_ = 0;
                lcp_[pos_[i]] = -1;
                continue;
            } else {
                for (IndexT &ptr = currentLcp_, j = suffixArray_[pos_[i] + 1];
                     std::max(i + ptr, j + ptr) < s_.size() && s_[i + ptr] == s_[j + ptr]; ++ptr) {}
                lcp_[pos_[i]] = currentLcp_;
            }
        }
        sparseTable = SparseTable(lcp_);
    }

    const std::string &s_;
    const std::vector<IndexT> &suffixArray_;

    std::vector<IndexT> lcp_;
    std::vector<IndexT> pos_;
    SparseTable<IndexT> sparseTable;

    void fillPos_() {
        for (int i = 0; i < s_.size(); ++i) {
            pos_[suffixArray_[i]] = i;
        }
    }
};

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

class RefrainReader {
public:
    void read(std::istream &in = std::cin) {
        in >> n_ >> m_;
        data_.resize(n_);
        for (auto &d : data_) {
            in >> d;
        }
    }

    int numElements() const {
        return n_;
    }

    int maxElement() const {
        return m_;
    }

    const std::vector<int> &getData() const {
        return data_;
    }

private:
    int n_{}, m_{};
    std::vector<int> data_;
};

class RefrainSolver {
protected:
    typedef int IndexT;
    typedef long long CountT;
public:
    explicit RefrainSolver(RefrainReader &reader) :
            reader_(reader) {
        convertData_();
    }

    virtual void solve() = 0;

    void printAns(std::ostream &out = std::cout) {
        out << ansLen_ * ansOccurrences_ << "\n" << ansLen_ << "\n";
        for (IndexT pos = ansPos_; pos < ansPos_ + ansLen_; ++pos) {
            out << reader_.getData()[pos] << " ";
        }
        out << "\n";
    }

    bool operator==(const RefrainSolver &solver) const {
        return ansLen_ * ansOccurrences_ == solver.ansLen_ * solver.ansOccurrences_;
    }

protected:
    RefrainReader &reader_;
    std::string str_;
    CountT ansLen_ = 0;
    CountT ansOccurrences_ = 0;
    CountT ansPos_ = 0;

    void convertData_() {
        str_.clear();
        for (auto &item : reader_.getData()) {
            str_ += char('a' + item - 1);
        }
    }
};

class RefrainSuffixArraySolver : public RefrainSolver {
public:
    explicit RefrainSuffixArraySolver(RefrainReader &reader) :
            RefrainSolver(reader) {}

    void solve() override {
        ansOccurrences_ = 1;
        ansLen_ = str_.size();
        SuffixArray suffixArray(str_ + '$');
        LCP lcp(suffixArray);
        auto &arrLcp = lcp.getLcp();
        std::stack<LcpEntity> lcpHeap_;
        lcpHeap_.emplace(-1, 0, 0);
        for (IndexT pos = 0; pos <= str_.size(); ++pos) {
            while (lcpHeap_.top().len > arrLcp[pos]) {
                tryUpdate_(lcpHeap_.top(), pos, suffixArray.getSuffixArray()[pos]);
                lcpHeap_.pop();
            }
            lcpHeap_.emplace(arrLcp[pos], pos, lcpHeap_.top().pos);
        }
    }

private:
    struct LcpEntity {
        CountT len, pos, maxLeft;

        LcpEntity(IndexT len, IndexT pos, IndexT maxLeft) :
                len(len), pos(pos), maxLeft(maxLeft) {}
    };

    void tryUpdate_(LcpEntity entity, CountT lastPos, CountT orig) {
        if (ansLen_ * ansOccurrences_ < entity.len * (lastPos - entity.maxLeft)) {
            ansLen_ = entity.len;
            ansPos_ = orig;
            ansOccurrences_ = lastPos - entity.maxLeft;
        }
    }
};

class RefrainSuffixTreeSolver : public RefrainSolver {
public:
    explicit RefrainSuffixTreeSolver(RefrainReader &reader) :
            RefrainSolver(reader) {}

    void solve() override {
        tree_ += str_ + '$';
        occur_.resize(tree_.size());
        calcOccurDfs_(tree_.root());
        findRefrainDfs_(tree_.root());
    }

private:
    typedef SuffixTree<> SuffTree;
    SuffTree tree_;
    std::vector<CountT> occur_;

    void calcOccurDfs_(const SuffTree::iterator &current) {
        if (current.isLeaf()) {
            occur_[current.ind()] = 1;
            return;
        }
        for (auto[letter, to] : current) {
            calcOccurDfs_(to);
            occur_[current.ind()] += occur_[to.ind()];
        }
    }

    void findRefrainDfs_(const SuffTree::iterator &current, CountT len = 0) {
        tryUpdate_(len + current.len(), occur_[current.ind()], current.l() - len);
        len += current.len();
        for (auto[letter, to] : current) {
            findRefrainDfs_(to, len);
        }
    }

    void tryUpdate_(CountT len, CountT occur, CountT pos) {
        if (ansLen_ * ansOccurrences_ < len * occur) {
            ansLen_ = len;
            ansOccurrences_ = occur;
            ansPos_ = pos;
        }
    }
};

class RefrainSuffixAutomatonSolver : public RefrainSolver {
public:
    explicit RefrainSuffixAutomatonSolver(RefrainReader &reader) :
            RefrainSolver(reader) {}

    void solve() override {
        SuffAutomaton automaton;
        automaton += str_;
        automaton.fillTerminals();
        used_.resize(automaton.size());
        used_.assign(automaton.size(), false);
        occur_.resize(automaton.size());
        calcOccurDfs_(automaton.root());
        used_.assign(automaton.size(), false);
        firstOccur_.resize(automaton.size());
        calcFirstOccurDfs_(automaton.root());
        used_.assign(automaton.size(), false);
        findRefrainDfs_(automaton.root());
    }

private:
    typedef SuffixAutomaton<> SuffAutomaton;
    std::vector<CountT> occur_;
    std::vector<CountT> firstOccur_;
    std::vector<bool> used_;

    void calcOccurDfs_(const SuffAutomaton::iterator &current) {
        used_[current.ind()] = true;
        if (current.isTerminal()) {
            occur_[current.ind()] = 1;
        }
        for (auto[letter, to] : current) {
            if (!used_[to.ind()]) {
                calcOccurDfs_(to);
            }
            occur_[current.ind()] += occur_[to.ind()];
        }
    }

    void calcFirstOccurDfs_(const SuffAutomaton::iterator &current) {
        used_[current.ind()] = true;
        firstOccur_[current.ind()] = str_.size();
        for (auto[letter, to] : current) {
            if (!used_[to.ind()]) {
                calcFirstOccurDfs_(to);
            }
            firstOccur_[current.ind()] = std::min(firstOccur_[current.ind()], firstOccur_[to.ind()] - 1);
        }
    }

    void findRefrainDfs_(const SuffAutomaton::iterator &current) {
        used_[current.ind()] = true;
        tryUpdate_(current.len(), occur_[current.ind()], firstOccur_[current.ind()] - current.len());
        for (auto[letter, to] : current) {
            if (!used_[to.ind()]) {
                findRefrainDfs_(to);
            }
        }
    }

    void tryUpdate_(CountT len, CountT occur, CountT pos) {
        if (ansLen_ * ansOccurrences_ < len * occur) {
            ansLen_ = len;
            ansOccurrences_ = occur;
            ansPos_ = pos;
        }
    }
};

int main() {
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);
    RefrainReader reader;
    reader.read();
    RefrainSuffixArraySolver arraySolver(reader);
    RefrainSuffixAutomatonSolver automatonSolver(reader);
    RefrainSuffixTreeSolver treeSolver(reader);
    arraySolver.solve();
    automatonSolver.solve();
    treeSolver.solve();
    assert(arraySolver == automatonSolver && automatonSolver == treeSolver);
    automatonSolver.printAns();
    return 0;
}