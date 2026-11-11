#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

class HungarianAlgorithm {
  public:
    static std::vector<int>
    solve(const std::vector<std::vector<double>> &cost_matrix,
          double max_allowed_cost = 0.6) {
        int n = cost_matrix.size();
        if (n == 0)
            return {};
        int m = cost_matrix[0].size();
        if (m == 0)
            return std::vector<int>(n, -1);

        int dim = std::max(n, m);
        const double PAD = 1e6;
        const double INF = std::numeric_limits<double>::infinity();
        std::vector<std::vector<double>> a(dim + 1,
                                           std::vector<double>(dim + 1, 0.0));

        for (int i = 0; i < dim; ++i) {
            for (int j = 0; j < dim; ++j) {
                if (i < n && j < m) {
                    a[i + 1][j + 1] = cost_matrix[i][j];
                } else {
                    a[i + 1][j + 1] = PAD;
                }
            }
        }

        std::vector<double> u(dim + 1, 0.0), v(dim + 1, 0.0);
        std::vector<int> p(dim + 1, 0), way(dim + 1, 0);

        for (int i = 1; i <= dim; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(dim + 1, INF);
            std::vector<char> used(dim + 1, false);

            do {
                used[j0] = true;
                int i0 = p[j0];
                double delta = INF;
                int j1 = 0;

                for (int j = 1; j <= dim; ++j) {
                    if (!used[j]) {
                        double cur = a[i0][j] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }

                for (int j = 0; j <= dim; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);

            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0 != 0);
        }

        std::vector<int> result(n, -1);
        for (int j = 1; j <= dim; ++j) {
            if (p[j] >= 1 && p[j] <= n) {
                int yolo_idx = p[j] - 1;
                int spaam_idx = j - 1;
                if (spaam_idx < m &&
                    cost_matrix[yolo_idx][spaam_idx] <= max_allowed_cost) {
                    result[yolo_idx] = spaam_idx;
                }
            }
        }
        return result;
    }
};
