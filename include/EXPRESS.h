#ifndef EXPRESS_H
#define EXPRESS_H
/**
 * This file is part of MOV-SLAM
 *
 * Copyright (C) 2022
 *
 * MOV-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * MOV-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with MOV-SLAM.
 * If not, see <http://www.gnu.org/licenses/>.
 */

static const int _8X8_L[15] = {1, 2, 3, 4, 5, 6, 7, 8, 7, 6, 5, 4, 3, 2, 1};
static const int _8X8_S[15] = {7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0};
static const int _8X8_R[2][15] = {{7, 7, 7, 7, 7, 7, 7, 7, 6, 5, 4, 3, 2, 1, 0},  // Right to Left
                                  {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7}}; // Bottom to Top

static const int _16X8_L[23] = {1, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 6, 5, 4, 3, 2, 1};
static const int _16X8_S[23] = {15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0};
static const int _16X8_R[2][23] = {{7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 5, 4, 3, 2, 1, 0},  // Right to Left
                                   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7}}; // Bottom to Top

static const int _8X16_L[23] = {1, 2, 3, 4, 5, 6, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 6, 5, 4, 3, 2, 1};
static const int _8X16_S[23] = {7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const int _8X16_R[2][23] = {{15, 15, 15, 15, 15, 15, 15, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}, // Right to Left
                                   {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}};       // Bottom to Top

static const int _16X16_L[31] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
static const int _16X16_S[31] = {15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const int _16X16_R[2][31] = {{15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0}, // Right to Left
                                    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15}};               // Bottom to Top

static cv::Mat diagonal(const cv::Mat &m_, const int8_t &d, const bool &direction)
{
    cv::Mat m = m_;
    size_t esz = m.elemSize();
    int len;

    if (m.rows == 8 && m.cols == 8)
    {
        len = _8X8_L[d];
        m.data += ((m.step[0] * _8X8_S[d]) + (esz * _8X8_R[direction][d]));
    }
    else if (m.rows == 16 && m.cols == 8)
    {
        len = _16X8_L[d];
        m.data += ((m.step[0] * _16X8_S[d]) + (esz * _16X8_R[direction][d]));
    }
    else if (m.rows == 8 && m.cols == 16)
    {
        len = _8X16_L[d];
        m.data += ((m.step[0] * _8X16_S[d]) + (esz * _8X16_R[direction][d]));
    }
    else if (m.rows == 16 && m.cols == 16)
    {
        len = _16X16_L[d];
        m.data += ((m.step[0] * _16X16_S[d]) + (esz * _16X16_R[direction][d]));
    }
    else
    {
        std::cout << "Unsupported diagonal matrix found, " << m_.rows << ", " << m.cols << std::endl;
    }

    m.size[0] = m.rows = len;
    m.size[1] = m.cols = 1;
    m.step[0] += direction ? esz : -esz;

    m.updateContinuityFlag();
    return m;
}

static uint8_t compute_center(const cv::Mat &img)
{
    uint8_t center_row = img.rows / 2;
    uint8_t center_col = img.cols / 2;
    return (img.at<uint8_t>(center_col, center_row) +
            img.at<uint8_t>(center_col - 1, center_row - 1) +
            img.at<uint8_t>(center_col, center_row - 1) +
            img.at<uint8_t>(center_col - 1, center_row)) /
           4;
}

static void compute_descriptor(cv::Mat &img, const int &threshold, bitset<256> &desc)
{
    uint8_t center = compute_center(img);
    uint8_t low_bounds = center - threshold;
    uint8_t high_bounds = center + threshold;

    desc.reset();

    for (int y = 0; y < img.rows; ++y)
    {
        uchar *p = img.ptr(y);
        for (int x = 0; x < img.cols; ++x)
        {
            p++;
            if ((low_bounds > *p) || (high_bounds < *p))
            {
                desc.set((y * img.rows) + x, true);
            }
        }
    }
}

static int compute_distance(const bitset<256> &desc1, const bitset<256> &desc2)
{
    return (desc1 ^ desc2).count();
}

static bool compute_express(cv::Mat &img, const int &threshold)
{
    uint8_t center = compute_center(img);
    uint8_t low_bounds = center - threshold;
    uint8_t high_bounds = center + threshold;
    uint8_t precheck = (img.rows * img.cols * .125);

    uint8_t f = 0;
    for (int row = 0; row < img.rows; ++row)
    {
        uchar *p = img.ptr(row);
        for (int col = 0; col < img.cols; ++col)
        {
            p++;
            if (low_bounds > *p || high_bounds < *p)
                f++;
        }
        if (f >= precheck)
            break;
    }

    if (f < precheck)
        return false;

    uint8_t slices = img.rows + img.cols - 1;
    uint8_t slices_half = (slices - 1) / 2;
    uint8_t rounds = round(slices * .25);
    uint8_t u_rounds = slices - rounds;
    uint8_t wins, losses, win, loss;

    for (int a = 0; a < 2; a++)
    {
        wins = 0;
        losses = 0;
        for (int i = 0; i < slices; i++)
        {
            cv::Mat diag = diagonal(img, i, a == 0);
            win = 0;
            loss = 0;
            for (int r = 0; r < diag.rows; r++)
            {
                if (low_bounds > *diag.ptr(r) || high_bounds < *diag.ptr(r))
                {
                    win++;
                }
                else
                {
                    loss++;
                }
            }
            if (wins < rounds)
            {
                if (win >= loss)
                    wins++;
                else
                    wins = 0;
            }

            if (losses < rounds)
            {
                if (loss > win)
                    losses++;
                else

                    losses = 0;
            }
            if (i > u_rounds && (wins == 0 || losses == 0))
                 break;
        }
        if (wins >= rounds && losses >= rounds)
        {
            return true;
        }
    }
    return false;
}

static inline void save_express_file(std::string file, const cv::Mat &_img, const int threshold)
{
    cv::Mat img(_img.rows, _img.cols, CV_8UC1, cv::Scalar(255));

    for (int y = 8; y < img.rows - 8; y++)
    {
        for (int x = 8; x < img.cols - 8; x++)
        {
            cv::Rect mb(x - 8, y - 8, 16, 16);
            if (mb.x >= 0 && mb.y >= 0 && (mb.x + mb.width) < img.cols && (mb.y + mb.height) < img.rows)
            {
                cv::Mat mb_img = _img(mb);
                bitset<256> desc;
                if (compute_express(mb_img, threshold))
                {
                    compute_descriptor(mb_img, threshold, desc);

                    int cnt = 0;
                    for (int i = mb.tl().y; i < mb.br().y; i++)
                    {
                        for (int j = mb.tl().x; j < mb.br().x; j++)
                        {
                            if (desc[cnt] == 1)
                            {
                                img.at<uint8_t>(i, j) = 0;
                            }
                            cnt++;
                        }
                    }
                }
            }
        }
    }
    std::cout << "Writing EXPRESS file " << file << std::endl;
    imwrite(file, img);
}

#endif