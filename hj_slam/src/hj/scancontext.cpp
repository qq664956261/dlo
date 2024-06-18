#include "hj/scancontext.h"

// namespace SC2
// {

void coreImportTest(void)
{
    cout << "scancontext lib is successfully imported." << endl;
} // coreImportTest

float rad2deg(float radians)
{
    return radians * 180.0 / M_PI;
}

float deg2rad(float degrees)
{
    return degrees * M_PI / 180.0;
}

float xy2theta(const float &_x, const float &_y)
{
    if ((_x >= 0) && (_y >= 0))
        return (180 / M_PI) * atan(_y / _x);

    if ((_x < 0) && (_y >= 0))
        return 180 - ((180 / M_PI) * atan(_y / (-_x)));

    if ((_x < 0) && (_y < 0))
        return 180 + ((180 / M_PI) * atan(_y / _x));

    if ((_x >= 0) && (_y < 0))
        return 360 - ((180 / M_PI) * atan((-_y) / _x));
    return 0.f;
} // xy2theta
// 输入
// 1 2 3
// 4 5 6
// 7 8 9
// 输出
// 3 1 2
// 6 4 5
// 9 7 8
MatrixXd circshift(MatrixXd &_mat, int _num_shift)
{
    // shift columns to right direction
    assert(_num_shift >= 0);

    if (_num_shift == 0)
    {
        MatrixXd shifted_mat(_mat);
        return shifted_mat; // Early return
    }

    MatrixXd shifted_mat = MatrixXd::Zero(_mat.rows(), _mat.cols());
    for (int col_idx = 0; col_idx < _mat.cols(); col_idx++)
    {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

} // circshift

std::vector<float> eigen2stdvec(MatrixXd _eigmat)
{
    std::vector<float> vec(_eigmat.data(), _eigmat.data() + _eigmat.size());
    return vec;
} // eigen2stdvec

// distDirectSC函数用于计算两个扫描上下文之间的直接相似度距离。扫描上下文是一种描述环境特征的数据结构
double SCManager::distDirectSC(MatrixXd &_sc1, MatrixXd &_sc2)
{

    // 初始化num_eff_cols为0，这个变量用来记录有效的列数（即不是全零的扇区）。
    int num_eff_cols = 0; // i.e., to exclude all-nonzero sector
    // sum_sector_similarity储存所有扇区（sector）相似度的总和。
    double sum_sector_similarity = 0;
    // 通过一个循环遍历矩阵的列（这里的每一列代表一个扇区），用col_idx作为索引。
    for (int col_idx = 0; col_idx < _sc1.cols(); col_idx++)
    {
        // 在每次循环中，分别取出两个扫描上下文矩阵中相同扇区的数据列，储存在VectorXd类型的col_sc1和col_sc2中。
        VectorXd col_sc1 = _sc1.col(col_idx);
        VectorXd col_sc2 = _sc2.col(col_idx);
        // 检查两个向量是否为零向量（即确定它们是否非空），如果有一个是零向量，则跳过此次循环，不计入相似度和有效列数。这样做是为了排除空白扇区，防止它们影响最终的相似度计算。
        if ((col_sc1.norm() == 0) | (col_sc2.norm() == 0))
            continue; // don't count this sector pair.
        // 通过点乘（.dot()）计算两列的余弦相似度，并标准化结果，使得相似度值在0到1之间。余弦相似度通过点乘结果除以两个矢量的范数乘积来计算。
        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());
        // 更新sum_sector_similarity和num_eff_cols，分别添加当前扇区的相似度和增加有效扇区数计数。
        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }
    // 循环结束后，计算平均相似度sc_sim，方法是将相似度总和sum_sector_similarity除以有效扇区数num_eff_cols
    double sc_sim = sum_sector_similarity / num_eff_cols;
    // std::cout<<"num_eff_cols: "<<num_eff_cols<<std::endl;
    // 最后，函数返回1.0 - sc_sim，这表示返回的值是差异度而不是相似度，差异度越低表示两个扫描上下文越相似
    return 1.0 - sc_sim;

} // distDirectSC

// 它通过对比两个矩阵（或者称为“变异键”或者“扇区键”，_vkey1 和 _vkey2）在进行一系列循环移位之后的相似性，
// 来计算它们之间的最优对齐方式。返回的是使得 _vkey1 和 _vkey2 之间差异最小的移位值
//  函数 fastAlignUsingVkey 旨在快速找到两个 variant key 矩阵 (_vkey1 和 _vkey2) 之间的最佳对齐方式。
//  Variant key 通常是一个描述了某种特征（比如扫描上下文中的旋转不变特征）的矩阵。
//  这个函数的输入是两个 MatrixXd 类型的矩阵 _vkey1 和 _vkey2，它们通常具有相同的列数，
//  每一列代表一个特定的方位角扇区。目标是通过在列方向上对 _vkey2 进行循环移位，来寻找最小化两个矩阵差异的对齐方式。
int SCManager::fastAlignUsingVkey(MatrixXd &_vkey1, MatrixXd &_vkey2)
{
    // 初始化一个变量 argmin_vkey_shift 保存最小差异对应的移位数，初始值设为 0。
    int argmin_vkey_shift = 0;
    // min_veky_diff_norm 变量用来存储目前为止观察到的最小差异范数，初始设一个很大的值以便于后续比较。
    double min_veky_diff_norm = 10000000;
    // 通过一个循环遍历可能的所有移位值，从 0 到 _vkey1 的列数减 1。
    for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++)
    {
        // 在每次迭代中，会调用 circshift 函数对矩阵 _vkey2 进行 shift_idx 数量的循环移位操作，产生 vkey2_shifted
        MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);
        // 计算 _vkey1 和 vkey2_shifted 之间的差异矩阵 vkey_diff。
        MatrixXd vkey_diff = _vkey1 - vkey2_shifted;
        // 通过调用 norm 方法计算该差异矩阵的范数（即 L2 范数或 Euclidean 范数），即 cur_diff_norm。
        double cur_diff_norm = vkey_diff.norm();
        // 如果这个范数小于当前记录的最小范数，就更新 argmin_vkey_shift 为当前的移位数，同时更新 min_veky_diff_norm 为这个较小的范数值。
        if (cur_diff_norm < min_veky_diff_norm)
        {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }
    // 循环完成后，返回对齐后的最小差异对应的移位数 argmin_vkey_shift。
    return argmin_vkey_shift;

} // fastAlignUsingVkey

// 其目的在于测量两个扫描上下文之间的距离，并找到最优的列（扇区）对齐方式，从而最小化两个上下文之间的差异。
std::pair<double, int> SCManager::distanceBtnScanContext(MatrixXd &_sc1, MatrixXd &_sc2)
{
    // 1. fast align using variant key (not in original IROS18)
    // 快速对齐变种键（variant key）：首先，通过makeSectorkeyFromScancontext函数从两个扫描上下文 _sc1 和 _sc2 中生成对应的变种键 vkey_sc1 和 vkey_sc2。
    // 这些变种键是一种降维表示，通常用于快速预对齐两个上下文。然后，通过fastAlignUsingVkey函数找到使两个变种键最为接近的最佳移位数 argmin_vkey_shift。
    MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
    MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
    int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);
    // 计算搜索半径：定义了一个搜索半径SEARCH_RADIUS，这是基于 _sc1 的列数和一个预先定义的比例 SEARCH_RATIO 计算得出的。
    // 这个搜索半径用于限定后面搜索最佳对齐位置时考虑的移位范围。
    const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _sc1.cols()); // a half of search range
    // 建立搜索空间：以 argmin_vkey_shift 为中心，构建一个包含从 -SEARCH_RADIUS 到 +SEARCH_RADIUS
    // 所有可能的移位值的搜索空间 shift_idx_search_space。这个搜索空间表示考虑在预对齐基础上进一步微调的所有可能移位数。
    std::vector<int> shift_idx_search_space{argmin_vkey_shift};
    for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++)
    {
        // 首先，argmin_vkey_shift + ii 会根据当前的迭代步骤 ii 计算出一个新的偏移位置。
        // 然后，为了确保这个新的偏移位置不会超出有效列范围（即不会计算出一个比 _sc1 的列数还要大的列索引），
        // 在它之前加上 _sc1.cols() 确保这个数字为正。最终，通过对 _sc1.cols() 取余数（% _sc1.cols()），
        // 确保结果位于 [0, _sc1.cols() - 1] 的范围内，这有效地实现了一种循环或者说“回绕”的效果。
        // 若移位计算结果超出了列数，就会从矩阵的另一边继续计数，模仿了循环的行为。
        shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff
    // 快速列差异（columnwise difference）：通过遍历shift_idx_search_space中的每一个移位值 num_shift，
    // 使用之前定义的 circshift 函数对 _sc2 进行移位，生成 sc2_shifted。
    // 接着，用distDirectSC函数计算_sc1与sc2_shifted之间的每一对应移位距离cur_sc_dist。
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for (int num_shift : shift_idx_search_space)
    {
        MatrixXd sc2_shifted = circshift(_sc2, num_shift);
        double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
        // 寻找最小距离：在所有可能的移位中寻找到使得 _sc1 和 sc2_shifted 之间距离最小的那一个，
        // 并记录这个最小距离min_sc_dist和对应的移位数argmin_shift。
        if (cur_sc_dist < min_sc_dist)
        {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }
    // 返回值：最后，函数返回这个最小距离和最优对齐移位值的一对值std::pair<double, int>。
    return make_pair(min_sc_dist, argmin_shift);

} // distanceBtnScanContext

// 用于从点云数据中生成一个叫做"Scan Context"的描述子。"Scan Context"是一种二维的极坐标表示法，它对环境的特定“视角”进行编码
MatrixXd SCManager::makeScancontext(pcl::PointCloud<SCPointType> &_scan_down)
{
    // 获得输入（降采样后）点云 _scan_down 中点的数量。
    int num_pts_scan_down = _scan_down.points.size();

    // main
    // 初始化一个描述子矩阵 desc，其尺寸为PC_NUM_RING行与PC_NUM_SECTOR列，并且初始值为 NO_POINT，表示没有点的位置
    const int NO_POINT = -1000;
    MatrixXd desc = NO_POINT * MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

    SCPointType pt;
    float azim_angle, azim_range; // wihtin 2d plane
    int ring_idx, sctor_idx;
    for (int pt_idx = 0; pt_idx < num_pts_scan_down; pt_idx++)
    {
        // 从点云中提取三维坐标x、y、z，并假设激光雷达的高度是一个固定值LIDAR_HEIGHT。
        pt.x = _scan_down.points[pt_idx].x;
        pt.y = _scan_down.points[pt_idx].y;
        pt.z = _scan_down.points[pt_idx].z + LIDAR_HEIGHT; // naive adding is ok (all points should be > 0).

        // xyz to ring, sector
        // 计算该点在水平面内的极坐标距离azim_range和角度azim_angle
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        // 检查点的范围是否超出预设的最大兴趣半径PC_MAX_RADIUS，如果超出则跳过该点。
        if (azim_range > PC_MAX_RADIUS)
            continue;
        // 通过简单比例关系将点的极坐标转换为描述子矩阵desc的行索引ring_idx和列索引sctor_idx。
        ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
        sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

        // taking maximum z
        // 更新描述子矩阵desc在该位置的值为点的z坐标，仅当新的z坐标大于已存在的值时才进行更新，这样能够保留最高点的高度作为区域特征。
        if (desc(ring_idx - 1, sctor_idx - 1) < pt.z) // -1 means cpp starts from 0
            desc(ring_idx - 1, sctor_idx - 1) = pt.z; // update for taking maximum value at that bin
    }
    // 完成描述子矩阵的构建后，遍历描述子矩阵将所有NO_POINT的值设置为0，为后续的余弦距离计算做准备。
    //  reset no points to zero (for cosine dist later)
    for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
        for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
            if (desc(row_idx, col_idx) == NO_POINT)
                desc(row_idx, col_idx) = 0;

    // 计时结束，并返回构建好的描述子矩阵 desc。
    return desc;
} // SCManager::makeScancontext

// 用于从扫描上下文描述子 _desc 生成所谓的环键（ring key）。环键是一个向量，它包含了 _desc 中每一行（即每一个环）的均值。
// 环键是一种特征表示，它对换向（例如机器人平移）具有不变性，因此在扫描上下文中能够帮助实现位置识别和对齐。
MatrixXd SCManager::makeRingkeyFromScancontext(Eigen::MatrixXd &_desc)
{
    /*
     * summary: rowwise mean vector
     */

    // 创建一个新的 Eigen::MatrixXd 类型的矩阵 invariant_key，具有与 _desc 相同数目的行和一列，初始化为默认值。
    Eigen::MatrixXd invariant_key(_desc.rows(), 1);
    //     遍历 _desc 每一行，用索引 row_idx：
    // a. 提取当前行作为一个向量 curr_row。
    // b. 计算 curr_row 的均值，并将结果赋值给 invariant_key 对应的元素。这里 invariant_key 的列索引始终为 0，因为我们生成的是一个列向量。
    for (int row_idx = 0; row_idx < _desc.rows(); row_idx++)
    {
        Eigen::MatrixXd curr_row = _desc.row(row_idx);
        invariant_key(row_idx, 0) = curr_row.mean();
    }
    // 循环结束后，返回 invariant_key。
    return invariant_key;
} // SCManager::makeRingkeyFromScancontext

// 作用是创建一个基于扫描上下文 _desc 矩阵的列向量平均值的特征向量，称为扇区键（sector key）。
// 这个向量包含了输入矩阵的每一列的均值，可以对扫描上下文矩阵的每个扇区进行编码。扇区键有助于在扫描匹配过程中捕捉环境中的方向性特征
MatrixXd SCManager::makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc)
{
    /*
     * summary: columnwise mean vector
     */

    // 初始化一个新的 Eigen::MatrixXd 类型的矩阵 variant_key，它是一个行向量，有一行和与 _desc 相同数目的列。
    Eigen::MatrixXd variant_key(1, _desc.cols());
    //     遍历 _desc 矩阵的每一列，使用列索引 col_idx：
    // a. 提取当前列为一个向量 curr_col。
    // b. 计算 curr_col 的均值，并将这个均值赋值到 variant_key 的相应列中。这里 variant_key 的行索引始终为 0，因为我们生成的是一个行向量。
    for (int col_idx = 0; col_idx < _desc.cols(); col_idx++)
    {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);
        variant_key(0, col_idx) = curr_col.mean();
    }
    // 循环结束之后，返回 variant_key
    return variant_key;
} // SCManager::makeSectorkeyFromScancontext

// 该函数 SCManager::getConstRefRecentSCD 提供了一个只读（const）引用，返回 SCManager
// 类内部维护的扫描上下文描述子（Scan Context Descriptor）集合中最新（或最后）一个元素。这种方式避免了复制数据，提高了效率。
const Eigen::MatrixXd &SCManager::getConstRefRecentSCD(void)
{
    return polarcontexts_.back();
}

void SCManager::saveScancontextAndKeys(Eigen::MatrixXd _scd)
{
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(_scd);
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(_scd);
    std::vector<float> polarcontext_invkey_vec = eigen2stdvec(ringkey);

    polarcontexts_.push_back(_scd);
    polarcontext_invkeys_.push_back(ringkey);
    polarcontext_vkeys_.push_back(sectorkey);
    polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);
} // SCManager::makeAndSaveScancontextAndKeys

void SCManager::makeAndSaveScancontextAndKeys(pcl::PointCloud<SCPointType> &_scan_down)
{
    Eigen::MatrixXd sc = makeScancontext(_scan_down); // v1
    Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(sc);
    Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(sc);
    std::vector<float> polarcontext_invkey_vec = eigen2stdvec(ringkey);

    polarcontexts_.push_back(sc);
    polarcontext_invkeys_.push_back(ringkey);
    polarcontext_vkeys_.push_back(sectorkey);
    polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);
} // SCManager::makeAndSaveScancontextAndKeys

void SCManager::setSCdistThres(double _new_thres)
{
    SC_DIST_THRES = _new_thres;
} // SCManager::setThres

void SCManager::setMaximumRadius(double _max_r)
{
    PC_MAX_RADIUS = _max_r;
} // SCManager::setMaximumRadius

std::pair<int, float> SCManager::detectLoopClosureIDBetweenSession(std::vector<float> &_curr_key, Eigen::MatrixXd &_curr_desc)
{
    // 初始设定：函数一开始将 loop_id 设为 -1，表示尚未找到环路闭合。如果最后 loop_id 仍然是 -1，则表示此次搜索没有找到有效的环路闭合。
    int loop_id{-1}; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

    auto &curr_key = _curr_key;
    auto &curr_desc = _curr_desc; // current observation (query)

    // step 0: if first, construct the tree in batch
    // 检查搜索树是否已经构建：
    if (!is_tree_batch_made) // run only once
    {
        // polarcontext_invkeys_to_search_是一个存储所有环键（invariant keys）的容器。首先清空它，随后将所有现有的环键从polarcontext_invkeys_mat_中复制到该容器中
        polarcontext_invkeys_to_search_.clear();
        polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end());
        // polarcontext_tree_batch_是一个智能指针，用来持有搜索树。使用reset方法释放先前可能存在的搜索树实例（如果有的话），然后通过std::make_unique为新的搜索树分配内存，并进行初始化。
        polarcontext_tree_batch_.reset();
        polarcontext_tree_batch_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */);

        is_tree_batch_made = true; // for running this block only once
    }
    // min_dist用于存储找到的最小距离，初始化为一个非常大的数值，以确保任何实际计算出的距离都会比它小，从而可以通过比较获得真正的最小距离
    double min_dist = 10000000; // init with somthing large
    // nn_align记录最佳匹配闭环候选的对齐位置（即扇区旋转偏移量），初始化为0。
    int nn_align = 0;
    // nn_idx记录具有最小距离闭环候选的索引，初始化为0。
    int nn_idx = 0;

    // step 1: knn search
    // 执行k-近邻搜索：利用当前观察的环键进行k-近邻搜索，找到与当前观察最接近的若干历史观察。
    // 初始化候选索引和距离容器：
    // 创建两个std::vector，一个用于存放最近邻候选的索引 (candidate_indexes)，另一个用于存储它们的距离平方 (out_dists_sqr)。
    std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE);
    std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);
    // 设置k-近邻搜索结果集：
    // nanoflann::KNNResultSet<float> 是一个模板类，用于存储k-近邻搜索的结果。它被初始化为搜索指定数量 (NUM_CANDIDATES_FROM_TREE) 的近邻。
    nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE);
    // 初始化搜索结果集：
    // k   nnsearch_result.init() 方法使用 candidate_indexes 和 out_dists_sqr的地址来初始化结果集。
    knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
    // 执行k-近邻搜索：
    // 调用 polarcontext_tree_batch_ 指针上搜索树的 findNeighbors 方法，传入搜索结果集、当前观察的环键（curr_key），以及用于搜索的相关参数（nanoflann::SearchParams(10)）。
    polarcontext_tree_batch_->index->findNeighbors(knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10)); // error here

    // step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
    // 逐对距离计算
    // 遍历NUM_CANDIDATES_FROM_TREE数量的候选项。NUM_CANDIDATES_FROM_TREE是先前通过k-NN搜索确定的候选数量上限。
    // 对于每一个候选项，通过candidate_indexes[candidate_iter_idx]获取其索引，并用此索引从polarcontexts_中提取相应的历史扫描上下文描述符polarcontext_candidate。
    // 使用distanceBtnScanContext函数计算当前扫描上下文描述符curr_desc与选出的历史扫描上下文描述符polarcontext_candidate之间的距离以及最佳对齐方式。该函数返回一个包含距离值和对齐索引的std::pair。
    // candidate_dist是当前候选项与查询上下文之间的距离，这个距离衡量了二者之间的相似度（或者差异）。
    // candidate_align则是对应于最小距离的对齐索引，即找到的最佳对齐方式。

    for (int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++)
    {
        MatrixXd polarcontext_candidate = polarcontexts_[candidate_indexes[candidate_iter_idx]];
        std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_desc, polarcontext_candidate);

        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;

        if (candidate_dist < min_dist)
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;

            nn_idx = candidate_indexes[candidate_iter_idx];
        }
    }

    // step 3: similarity threshold
    if (min_dist < SC_DIST_THRES)
        loop_id = nn_idx;

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
    std::pair<int, float> result{loop_id, yaw_diff_rad};

    return result;

} // SCManager::detectLoopClosureIDBetweenSession

std::pair<int, float> SCManager::detectLoopClosureID(void)
{
    int loop_id{-1}; // init with -1, -1 means no loop (== LeGO-LOAM's variable "closestHistoryFrameID")

    auto curr_key = polarcontext_invkeys_mat_.back(); // current observation (query)
    auto curr_desc = polarcontexts_.back();           // current observation (query)

    /*
     * step 1: candidates from ringkey tree_ 
     */
    if ((int)polarcontext_invkeys_mat_.size() < NUM_EXCLUDE_RECENT + 1)
    {
        std::pair<int, float> result{loop_id, 0.0};
        return result; // Early return
    }

    // tree_ reconstruction (not mandatory to make everytime)
    if (tree_making_period_conter % TREE_MAKING_PERIOD_ == 0) // to save computation cost
    {

        polarcontext_invkeys_to_search_.clear();
        polarcontext_invkeys_to_search_.assign(polarcontext_invkeys_mat_.begin(), polarcontext_invkeys_mat_.end() - NUM_EXCLUDE_RECENT);

        polarcontext_tree_.reset();
        polarcontext_tree_ = std::make_unique<InvKeyTree>(PC_NUM_RING /* dim */, polarcontext_invkeys_to_search_, 10 /* max leaf */);
        // tree_ptr_->index->buildIndex(); // inernally called in the constructor of InvKeyTree (for detail, refer the nanoflann and KDtreeVectorOfVectorsAdaptor)
    }
    tree_making_period_conter = tree_making_period_conter + 1;

    double min_dist = 10000000; // init with somthing large
    int nn_align = 0;
    int nn_idx = 0;

    // knn search
    // std::vector<size_t> candidate_indexes(NUM_CANDIDATES_FROM_TREE);
    // std::vector<float> out_dists_sqr(NUM_CANDIDATES_FROM_TREE);

    // nanoflann::KNNResultSet<float> knnsearch_result(NUM_CANDIDATES_FROM_TREE);
    // knnsearch_result.init(&candidate_indexes[0], &out_dists_sqr[0]);
    // polarcontext_tree_->index->findNeighbors(knnsearch_result, &curr_key[0] /* query */, nanoflann::SearchParams(10));

    /*
     *  step 2: pairwise distance (find optimal columnwise best-fit using cosine distance)
     */

    // for( int candidate_iter_idx = 0; candidate_iter_idx < NUM_CANDIDATES_FROM_TREE; candidate_iter_idx++ )
    for (size_t candidate_iter_idx = 0; candidate_iter_idx < polarcontexts_.size() - 1; candidate_iter_idx++)
    {
        //std::cout << "candidate_iter_idx:" << candidate_iter_idx << std::endl;
        // MatrixXd polarcontext_candidate = polarcontexts_[ candidate_indexes[candidate_iter_idx] ];
        MatrixXd polarcontext_candidate = polarcontexts_[candidate_iter_idx];
        // std::cout<<"curr_desc:"<<curr_desc<<std::endl;
        // std::cout<<"polarcontext_candidate:"<<polarcontext_candidate<<std::endl;
        std::pair<double, int> sc_dist_result = distanceBtnScanContext(curr_desc, polarcontext_candidate);

        double candidate_dist = sc_dist_result.first;
        int candidate_align = sc_dist_result.second;
        //std::cout << "candidate_dist:" << candidate_dist << std::endl;
        if (candidate_dist < min_dist)
        {
            min_dist = candidate_dist;
            nn_align = candidate_align;

            // nn_idx = candidate_indexes[candidate_iter_idx];
            nn_idx = candidate_iter_idx;
        }
    }
    std::cout << "nn_idx:" << nn_idx << std::endl;
    std::cout << "min_dist:" << min_dist << std::endl;
    Eigen::Matrix<float, 200, 1> e_curr_key;

    /*
     * loop threshold check
     */
    if (min_dist < SC_DIST_THRES)
    {
        loop_id = nn_idx;

        // std::cout.precision(3);
        cout << "[Loop found] Nearest distance: " << min_dist << " btn " << polarcontexts_.size() - 1 << " and " << nn_idx << "." << endl;
        // cout << "[Loop found] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }
    else
    {
        std::cout.precision(3);
        cout << "[Not loop] Nearest distance: " << min_dist << " btn " << polarcontexts_.size() - 1 << " and " << nn_idx << "." << endl;
        // cout << "[Not loop] yaw diff: " << nn_align * PC_UNIT_SECTORANGLE << " deg." << endl;
    }

    // To do: return also nn_align (i.e., yaw diff)
    float yaw_diff_rad = deg2rad(nn_align * PC_UNIT_SECTORANGLE);
    std::cout << "yaw_diff_rad:" << yaw_diff_rad << std::endl;
    std::pair<int, float> result{loop_id, yaw_diff_rad};

    return result;

} // SCManager::detectLoopClosureID

void SCManager::pop()
{
    polarcontexts_.pop_back();
    polarcontext_invkeys_.pop_back();
    polarcontext_vkeys_.pop_back();
    polarcontext_invkeys_mat_.pop_back();
}

// } // namespace SC2