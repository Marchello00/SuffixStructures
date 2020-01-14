#ifndef REFRAINSOLVER_SUFFIX_ARRAY_H
#define REFRAINSOLVER_SUFFIX_ARRAY_H

#include <iostream>
#include <vector>
#include <numeric>
#include <stack>
#include <cassert>
#include <memory>
#include "node.h"
#include "sparse_table.h"

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

#endif //REFRAINSOLVER_SUFFIX_ARRAY_H
