#include "Association.h"
#include "lapjv.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace motrack {

std::vector<STrackPtr> jointStracks(const std::vector<STrackPtr> &a_tlist,
                                    const std::vector<STrackPtr> &b_tlist)
{
    std::unordered_set<size_t> seen;
    seen.reserve(a_tlist.size() + b_tlist.size());

    std::vector<STrackPtr> res;
    res.reserve(a_tlist.size() + b_tlist.size());

    for (const auto &t : a_tlist)
    {
        seen.insert(t->getTrackId());
        res.push_back(t);
    }
    for (const auto &t : b_tlist)
    {
        if (seen.insert(t->getTrackId()).second)
        {
            res.push_back(t);
        }
    }
    return res;
}

std::vector<STrackPtr> subStracks(const std::vector<STrackPtr> &a_tlist,
                                  const std::vector<STrackPtr> &b_tlist)
{
    std::unordered_set<size_t> remove_ids;
    remove_ids.reserve(b_tlist.size());
    for (const auto &t : b_tlist)
    {
        remove_ids.insert(t->getTrackId());
    }

    std::vector<STrackPtr> res;
    res.reserve(a_tlist.size());
    for (const auto &t : a_tlist)
    {
        if (remove_ids.count(t->getTrackId()) == 0)
        {
            res.push_back(t);
        }
    }
    return res;
}

std::vector<std::vector<float>> calcIouDistance(const std::vector<STrackPtr> &a_tracks,
                                                const std::vector<STrackPtr> &b_tracks)
{
    if (a_tracks.empty() || b_tracks.empty())
    {
        return {};
    }

    std::vector<std::vector<float>> cost(a_tracks.size(),
                                         std::vector<float>(b_tracks.size(), 1.0f));

    for (size_t i = 0; i < a_tracks.size(); ++i)
    {
        const auto &oa = a_tracks[i]->getObject();
        const auto &ra = oa.rect;
        const float area_a = ra.width * ra.height;

        for (size_t j = 0; j < b_tracks.size(); ++j)
        {
            const auto &ob = b_tracks[j]->getObject();
            if (oa.label != ob.label)
            {
                continue;   // cost stays at 1.0 (max distance)
            }
            const auto &rb = ob.rect;

            const float left   = std::max(ra.x, rb.x);
            const float right  = std::min(ra.x + ra.width,  rb.x + rb.width);
            const float top    = std::max(ra.y, rb.y);
            const float bottom = std::min(ra.y + ra.height, rb.y + rb.height);
            if (right <= left || bottom <= top)
            {
                continue;
            }

            const float inter = (right - left) * (bottom - top);
            const float iou   = inter / (area_a + rb.width * rb.height - inter);
            cost[i][j] = 1.0f - iou;
        }
    }
    return cost;
}

void removeDuplicateStracks(const std::vector<STrackPtr> &a_stracks,
                            const std::vector<STrackPtr> &b_stracks,
                            std::vector<STrackPtr> &a_res,
                            std::vector<STrackPtr> &b_res)
{
    const auto dists = calcIouDistance(a_stracks, b_stracks);

    std::vector<bool> a_dup(a_stracks.size(), false);
    std::vector<bool> b_dup(b_stracks.size(), false);

    for (size_t i = 0; i < dists.size(); ++i)
    {
        for (size_t j = 0; j < dists[i].size(); ++j)
        {
            if (dists[i][j] >= 0.15f)
            {
                continue;
            }
            const size_t tp = a_stracks[i]->getFrameId() - a_stracks[i]->getStartFrameId();
            const size_t tq = b_stracks[j]->getFrameId() - b_stracks[j]->getStartFrameId();
            (tp > tq ? b_dup[j] : a_dup[i]) = true;
        }
    }

    a_res.reserve(a_stracks.size());
    for (size_t i = 0; i < a_stracks.size(); ++i)
    {
        if (!a_dup[i]) a_res.push_back(a_stracks[i]);
    }

    b_res.reserve(b_stracks.size());
    for (size_t j = 0; j < b_stracks.size(); ++j)
    {
        if (!b_dup[j]) b_res.push_back(b_stracks[j]);
    }
}

namespace {

double execLapjv(const std::vector<std::vector<float>> &cost,
                 std::vector<int> &rowsol,
                 std::vector<int> &colsol,
                 bool extend_cost,
                 float cost_limit,
                 bool return_cost = true)
{
    std::vector<std::vector<float> > cost_c;
    cost_c.assign(cost.begin(), cost.end());

    std::vector<std::vector<float> > cost_c_extended;

    int n_rows = cost.size();
    int n_cols = cost[0].size();
    rowsol.resize(n_rows);
    colsol.resize(n_cols);

    int n = 0;
    if (n_rows == n_cols)
    {
        n = n_rows;
    }
    else
    {
        if (!extend_cost)
        {
            throw std::runtime_error("The `extend_cost` variable should set True");
        }
    }

    if (extend_cost || cost_limit < std::numeric_limits<float>::max())
    {
        n = n_rows + n_cols;
        cost_c_extended.resize(n);
        for (size_t i = 0; i < cost_c_extended.size(); i++)
            cost_c_extended[i].resize(n);

        if (cost_limit < std::numeric_limits<float>::max())
        {
            for (size_t i = 0; i < cost_c_extended.size(); i++)
            {
                for (size_t j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_limit / 2.0;
                }
            }
        }
        else
        {
            float cost_max = -1;
            for (size_t i = 0; i < cost_c.size(); i++)
            {
                for (size_t j = 0; j < cost_c[i].size(); j++)
                {
                    if (cost_c[i][j] > cost_max)
                        cost_max = cost_c[i][j];
                }
            }
            for (size_t i = 0; i < cost_c_extended.size(); i++)
            {
                for (size_t j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_max + 1;
                }
            }
        }

        for (size_t i = n_rows; i < cost_c_extended.size(); i++)
        {
            for (size_t j = n_cols; j < cost_c_extended[i].size(); j++)
            {
                cost_c_extended[i][j] = 0;
            }
        }
        for (int i = 0; i < n_rows; i++)
        {
            for (int j = 0; j < n_cols; j++)
            {
                cost_c_extended[i][j] = cost_c[i][j];
            }
        }

        cost_c.clear();
        cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
    }

    double **cost_ptr;
    cost_ptr = new double *[sizeof(double *) * n];
    for (int i = 0; i < n; i++)
        cost_ptr[i] = new double[sizeof(double) * n];

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cost_ptr[i][j] = cost_c[i][j];
        }
    }

    int* x_c = new int[sizeof(int) * n];
    int *y_c = new int[sizeof(int) * n];

    int ret = lapjv_internal(n, cost_ptr, x_c, y_c);

    // Extract the solution (and optional cost) before releasing the buffers,
    // so the error path below can throw without leaking them.
    double opt = 0.0;
    if (ret == 0)
    {
        if (n != n_rows)
        {
            for (int i = 0; i < n; i++)
            {
                if (x_c[i] >= n_cols)
                    x_c[i] = -1;
                if (y_c[i] >= n_rows)
                    y_c[i] = -1;
            }
            for (int i = 0; i < n_rows; i++)
            {
                rowsol[i] = x_c[i];
            }
            for (int i = 0; i < n_cols; i++)
            {
                colsol[i] = y_c[i];
            }

            if (return_cost)
            {
                for (size_t i = 0; i < rowsol.size(); i++)
                {
                    if (rowsol[i] != -1)
                    {
                        opt += cost_ptr[i][rowsol[i]];
                    }
                }
            }
        }
        else if (return_cost)
        {
            for (size_t i = 0; i < rowsol.size(); i++)
            {
                opt += cost_ptr[i][rowsol[i]];
            }
        }
    }

    for (int i = 0; i < n; i++)
    {
        delete[]cost_ptr[i];
    }
    delete[]cost_ptr;
    delete[]x_c;
    delete[]y_c;

    if (ret != 0)
    {
        throw std::runtime_error("The result of lapjv_internal() is invalid.");
    }

    return opt;
}

} // namespace

void linearAssignment(const std::vector<std::vector<float>> &cost_matrix,
                                               const int &cost_matrix_size,
                                               const int &cost_matrix_size_size,
                                               const float &thresh,
                                               std::vector<std::vector<int>> &matches,
                                               std::vector<int> &a_unmatched,
                                               std::vector<int> &b_unmatched)
{
    if (cost_matrix.size() == 0)
    {
        a_unmatched.reserve(cost_matrix_size);
        for (int i = 0; i < cost_matrix_size; i++)
        {
            a_unmatched.push_back(i);
        }
        b_unmatched.reserve(cost_matrix_size_size);
        for (int i = 0; i < cost_matrix_size_size; i++)
        {
            b_unmatched.push_back(i);
        }
        return;
    }

    std::vector<int> rowsol; std::vector<int> colsol;
    execLapjv(cost_matrix, rowsol, colsol, true, thresh);

    matches.reserve(std::min(rowsol.size(), colsol.size()));
    a_unmatched.reserve(rowsol.size());
    b_unmatched.reserve(colsol.size());

    for (size_t i = 0; i < rowsol.size(); i++)
    {
        if (rowsol[i] >= 0)
        {
            matches.push_back({static_cast<int>(i), rowsol[i]});
        }
        else
        {
            a_unmatched.push_back(i);
        }
    }

    for (size_t i = 0; i < colsol.size(); i++)
    {
        if (colsol[i] < 0)
        {
            b_unmatched.push_back(i);
        }
    }
}

}
