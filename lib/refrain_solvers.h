#ifndef REFRAINSOLVER_REFRAIN_SOLVERS_H
#define REFRAINSOLVER_REFRAIN_SOLVERS_H

#include <iostream>
#include <vector>
#include <numeric>
#include <stack>
#include <cassert>
#include <memory>
#include "node.h"
#include "suffix_array.h"
#include "suffix_tree.h"
#include "suffix_automaton.h"

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

#endif //REFRAINSOLVER_REFRAIN_SOLVERS_H
