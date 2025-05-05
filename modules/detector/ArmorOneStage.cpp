#include "ArmorOneStage.hpp"
#include <Eigen/Core>
#include "opencv2/opencv.hpp"
#include "TimeStamp/TimeStamp.hpp"
#include "Log/log.hpp"

#define USE_OLD_MODEL false

namespace detector
{
    //params
    static constexpr int INPUT_W = 416;   // Width of input
    static constexpr int INPUT_H = 416;   // Height of input
//    static constexpr int NUM_CLASSES = 8; // Number of classes
//    static constexpr int NUM_COLORS = 8;  // Number of color
    static constexpr int NUM_CLASSES = USE_OLD_MODEL? 8 : 1;
    static constexpr int NUM_COLORS = USE_OLD_MODEL? 8 : 2;  // Number of color
    static constexpr int TOPK = 128;      // TopK
    static constexpr float NMS_THRESH = 0.3;
    static constexpr float BBOX_CONF_THRESH = 0.5;
    static constexpr float MERGE_CONF_ERROR = 0.15;
    static constexpr float MERGE_MIN_IOU = 0.9;

    ArmorOneStage::ArmorOneStage(const std::string &model_file, bool allowGray)
        : allowGray(allowGray)
    {
        initModel(model_file);
        number_classifier = std::make_unique<NumberClassifier>("svm");
    }

    ArmorOneStage::~ArmorOneStage()
    {
        
    }

    void ArmorOneStage::initModel(const std::string &model_file)
    {
        core.set_property("CPU", ov::enable_profiling(true));
        model = core.read_model(model_file);
        ov::preprocess::PrePostProcessor ppp(model);
        ppp.input().tensor().set_element_type(ov::element::f32);
        ppp.output().tensor().set_element_type(ov::element::f32);
        ppp.build();
        compiled_model = core.compile_model(
            model,
            "CPU",
            ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
            ov::inference_num_threads(4)//,
            //ov::hint::enable_cpu_pinning(true),
            //ov::hint::enable_hyper_threading(true)
        );
        infer_request = compiled_model.create_infer_request();
    }

    static inline int argmax(const float *ptr, int len)
    {
        int max_arg = 0;
        for (int i = 1; i < len; i++)
        {
            if (ptr[i] > ptr[max_arg])
                max_arg = i;
        }
        return max_arg;
    }

    inline cv::Mat scaledResize(const cv::Mat &img, Eigen::Matrix<float, 3, 3> &transform_matrix)
    {
        float r = std::min(INPUT_W / (img.cols * 1.0), INPUT_H / (img.rows * 1.0));
        int unpad_w = r * img.cols;
        int unpad_h = r * img.rows;

        int dw = INPUT_W - unpad_w;
        int dh = INPUT_H - unpad_h;

        dw /= 2;
        dh /= 2;

        transform_matrix << 1.0 / r, 0, -dw / r,
            0, 1.0 / r, -dh / r,
            0, 0, 1;

        cv::Mat re;
        //INFO("resize: {}x{} -> {}x{}", img.cols, img.rows, unpad_w, unpad_h);
        cv::resize(img, re, cv::Size(unpad_w, unpad_h));
        cv::Mat out;
        cv::copyMakeBorder(re, out, dh, dh, dw, dw, cv::BORDER_CONSTANT);
        return out;
    }

    static void generate_grids_and_stride(const int target_w, const int target_h, std::vector<int> &strides, std::vector<GridAndStride> &grid_strides)
    {
        for (auto stride : strides)
        {
            int num_grid_w = target_w / stride;
            int num_grid_h = target_h / stride;

            for (int g1 = 0; g1 < num_grid_h; g1++)
            {
                for (int g0 = 0; g0 < num_grid_w; g0++)
                {
                    GridAndStride grid_stride = {g0, g1, stride};
                    grid_strides.emplace_back(grid_stride);
                }
            }
        }
    }

    static void generateYoloxProposals(std::vector<GridAndStride> grid_strides, const float *feat_ptr,
                                       Eigen::Matrix<float, 3, 3> &transform_matrix, float prob_threshold,
                                       BBoxes &bboxes,int& color_flag, bool& allowGray)
    {
        const int num_anchors = grid_strides.size();
        // Travel all the anchors
        for (int anchor_idx = 0; anchor_idx < num_anchors; anchor_idx++)
        {
            const int grid0 = grid_strides[anchor_idx].grid0;
            const int grid1 = grid_strides[anchor_idx].grid1;
            const int stride = grid_strides[anchor_idx].stride;
            const int basic_pos = anchor_idx * (9 + (NUM_COLORS) + NUM_CLASSES);

            // yolox/models/yolo_head.py decode logic
            float x_1 = (feat_ptr[basic_pos + 0] + grid0) * stride;
            float y_1 = (feat_ptr[basic_pos + 1] + grid1) * stride;
            float x_2 = (feat_ptr[basic_pos + 2] + grid0) * stride;
            float y_2 = (feat_ptr[basic_pos + 3] + grid1) * stride;
            float x_3 = (feat_ptr[basic_pos + 4] + grid0) * stride;
            float y_3 = (feat_ptr[basic_pos + 5] + grid1) * stride;
            float x_4 = (feat_ptr[basic_pos + 6] + grid0) * stride;
            float y_4 = (feat_ptr[basic_pos + 7] + grid1) * stride;

            float box_objectness = (feat_ptr[basic_pos + 8]);
            int box_color = argmax(feat_ptr + basic_pos + 9, NUM_COLORS);
            int box_class = argmax(feat_ptr + basic_pos + 9 + NUM_COLORS, NUM_CLASSES);

            float box_prob = box_objectness;
            if (box_prob >= prob_threshold)
            {
                BBox bbox;

                Eigen::Matrix<float, 3, 4> apex_norm;
                Eigen::Matrix<float, 3, 4> apex_dst;

                apex_norm << x_1, x_2, x_3, x_4,
                    y_1, y_2, y_3, y_4,
                    1, 1, 1, 1;

                apex_dst = transform_matrix * apex_norm;

                for (int i = 0; i < 4; i++)
                {
                    bbox.corners[i] = cv::Point2f(apex_dst(0, i), apex_dst(1, i));
                    bbox.points.push_back(bbox.corners[i]);
                }

                std::vector<cv::Point2f> tmp(bbox.corners, bbox.corners + 4);
                bbox.rect = cv::boundingRect(tmp);
                bbox.tag_id = box_class;
                bbox.color_id = box_color;
                bbox.confidence = box_prob;
                bbox.area = bbox.rect.area();
                //if(box_color/2 == color_flag )//|| (allowGray && (box_color/2 == 2)))
                if(((!USE_OLD_MODEL) && (box_color == color_flag) )||(USE_OLD_MODEL&&(box_color/2 == color_flag)))
                {
                    bboxes.push_back(bbox);
                }
            }
        } // point anchor loop
    }


    static void qsort_descent_inplace(BBoxes &facebboxes, int left, int right)
    {
        int i = left;
        int j = right;
        float p = facebboxes[(left + right) / 2].confidence;

        while (i <= j)
        {
            while (facebboxes[i].confidence > p)
                i++;

            while (facebboxes[j].confidence < p)
                j--;

            if (i <= j)
            {
                // swap
                std::swap(facebboxes[i], facebboxes[j]);
                i++;
                j--;
            }
        }
        if (left < j)
            qsort_descent_inplace(facebboxes, left, j);
        if (i < right)
            qsort_descent_inplace(facebboxes, i, right);
    }

    static void qsort_descent_inplace(BBoxes &bboxes)
    {
        if (bboxes.empty())
            return;

        qsort_descent_inplace(bboxes, 0, bboxes.size() - 1);
    }

    static inline float intersection_area(const BBox &a, const BBox &b)
    {
        cv::Rect_<float> inter = a.rect & b.rect;
        return inter.area();
    }

    static void nms_sorted_bboxes(BBoxes &faceobjects, std::vector<int> &picked, float nms_threshold)
    {
        picked.clear();
        const int n = faceobjects.size();

        std::vector<float> areas(n);
        for (int i = 0; i < n; i++)
        {
            areas[i] = faceobjects[i].rect.area();
        }

        for (int i = 0; i < n; i++)
        {
            BBox &a = faceobjects[i];
            int keep = 1;
            for (int j = 0; j < (int)picked.size(); j++)
            {
                BBox &b = faceobjects[picked[j]];
                // intersection over union
                float inter_area = intersection_area(a, b);
                float union_area = areas[i] + areas[picked[j]] - inter_area;
                float iou = inter_area / union_area;

                if (iou > nms_threshold || isnan(iou))
                {
                    keep = 0;
                    // Stored for Merge
                    if (iou > MERGE_MIN_IOU && 
                        abs(a.confidence - b.confidence) < MERGE_CONF_ERROR && 
                        a.tag_id == b.tag_id &&    
                        a.color_id == b.color_id)
                    {
                        for (int i = 0; i < 4; i++)
                        {
                            b.points.push_back(a.corners[i]);
                        }
                    }
                }
            }
            if (keep)
                picked.push_back(i);
        }
    }


    BBoxes decodeOutputs(const float *prob, BBoxes &objects, Eigen::Matrix<float, 3, 3> &transform_matrix, int& color_flag, bool& allowGray)
    {
        BBoxes proposals;
        std::vector<int> strides = {8, 16, 32};
        std::vector<GridAndStride> grid_strides;

        generate_grids_and_stride(INPUT_W, INPUT_H, strides, grid_strides);
        generateYoloxProposals(grid_strides, prob, transform_matrix, BBOX_CONF_THRESH, proposals, color_flag, allowGray);
        qsort_descent_inplace(proposals);
        
        if (proposals.size() >= TOPK)
            proposals.resize(TOPK);
        std::vector<int> picked;
        nms_sorted_bboxes(proposals, picked, NMS_THRESH);
        int count = picked.size();
        objects.resize(count);
        for (int i = 0; i < count; i++)
        {
            objects[i] = proposals[picked[i]];
        }

        return proposals;
    }

    BBoxes ArmorOneStage::operator()(const cv::Mat &img)
    {
        Time::TimeStamp time_stamp = Time::TimeStamp::now();
        cv::Mat pre_img = scaledResize(img, transfrom_matrix);
        cv::Mat pre;
        cv::Mat pre_split[3];
        pre_img.convertTo(pre, CV_32F);
        cv::split(pre, pre_split);
        //INFO("ArmorOneStage pre_img size: {}x{}", pre_img.cols, pre_img.rows);
        input_tensor = infer_request.get_input_tensor(0);
        infer_request.set_input_tensor(input_tensor);
        float *tensor_data = input_tensor.data<float_t>();
        auto img_offset = INPUT_H * INPUT_W;
        for (int c = 0; c < 3; c++)
        {
            memcpy(tensor_data, pre_split[c].data, INPUT_H * INPUT_W * sizeof(float));
            tensor_data += img_offset;
        }
        infer_request.infer();
        ov::Tensor output_tensor = infer_request.get_output_tensor();
        float *output = output_tensor.data<float_t>();
        //INFO("ArmorOneStage output_tensor size: {}x{}", output_tensor.get_shape()[0], output_tensor.get_shape()[1]);
        BBoxes bboxes;
        BBoxes proposals = decodeOutputs(output, bboxes, transfrom_matrix, this->color_flag, this->allowGray);
	    INFO("ArmorOneStage decodeOutputs size: {}x{}", proposals.size(), bboxes.size());
        //following code is generated by AI without any check
        //carefully use it 
        BBoxes bboxes_temp;
        for (auto bbox = bboxes.begin(); bbox != bboxes.end(); ++bbox)
        {
            // 对候选框预测角点进行平均,降低误差
            if ((*bbox).points.size() >= 8)
            {
                auto N = (*bbox).points.size();
                // 初始化为0
                cv::Point2f pts_final[4] = {cv::Point2f(0,0), cv::Point2f(0,0), cv::Point2f(0,0), cv::Point2f(0,0)};
                for (int i = 0; i < (int)N; i++)
                {
                    pts_final[i % 4] += (*bbox).points[i];
                }

                for (int i = 0; i < 4; i++)
                {
                    pts_final[i].x = pts_final[i].x / (N / 4);
                    pts_final[i].y = pts_final[i].y / (N / 4);
                }
                (*bbox).center=((*bbox).corners[0]+(*bbox).corners[1]+(*bbox).corners[2]+(*bbox).corners[3])/4;
            }
            else
            {
                (*bbox).center=((*bbox).corners[0]+(*bbox).corners[1]+(*bbox).corners[2]+(*bbox).corners[3])/4;
            }

            USE_OLD_MODEL?(std::swap((*bbox).corners[1],(*bbox).corners[3]),1):0;
            
            // 检查角点是否在图像范围内
            bool valid_corners = true;
            for (int i = 0; i < 4; i++) {
                if ((*bbox).corners[i].x < 0 || (*bbox).corners[i].x >= img.cols ||
                    (*bbox).corners[i].y < 0 || (*bbox).corners[i].y >= img.rows) {
                    valid_corners = false;
                    INFO("检测到超出图像范围的角点: ({}, {}), 图像尺寸: {}x{}", 
                        (*bbox).corners[i].x, (*bbox).corners[i].y, img.cols, img.rows);
                    break;
                }
            }
            
            if (!valid_corners) {
                INFO("跳过包含无效角点的边界框");
                continue;
            }
            
            // 添加异常处理
            try {
                std::pair<int, double> result = number_classifier->predict(img, {(*bbox).corners[0], (*bbox).corners[1], (*bbox).corners[2], (*bbox).corners[3]});
                (*bbox).tag_id = result.first;

                if(result.first != 0)
                {
                    bboxes_temp.push_back(*bbox);
                }
            } catch (const cv::Exception& e) {
                WARN("数字分类器预测时发生异常: {}", e.what());
                // 跳过这个边界框
            } catch (const std::exception& e) {
                WARN("处理边界框时发生异常: {}", e.what());
                // 跳过这个边界框
            }
        }
        INFO("Armor Detect cost time: {}ms", (Time::TimeStamp::now() - time_stamp).toSeconds() * 1000);
//         //debug:
        for (auto bbox = bboxes_temp.begin(); bbox != bboxes_temp.end(); ++bbox)
        {
            //use corner
            cv::line(img, bbox->corners[0], bbox->corners[1], cv::Scalar(0, 255, 0), 2);
            cv::line(img, bbox->corners[1], bbox->corners[2], cv::Scalar(0, 255, 0), 2);
            cv::line(img, bbox->corners[2], bbox->corners[3], cv::Scalar(0, 255, 0), 2);
            cv::line(img, bbox->corners[3], bbox->corners[0], cv::Scalar(0, 255, 0), 2);
            //use center
            cv::circle(img, bbox->center, 3, cv::Scalar(0, 255, 0), -1);
        }
//        //cv::namedWindow("result",cv::WINDOW_NORMAL);
//        //cv::imshow("result",temp);
//        //cv::waitKey(1);


        return bboxes_temp;
    }

    

} // namespace DETECTOR
