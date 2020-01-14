#ifndef REFRAINSOLVER_SPARSE_TABLE_H
#define REFRAINSOLVER_SPARSE_TABLE_H

#include <iostream>
#include <vector>
#include <numeric>

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

#endif //REFRAINSOLVER_SPARSE_TABLE_H
