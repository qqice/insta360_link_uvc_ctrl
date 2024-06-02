//
// Created by qqice on 24-6-3.
//

#include "upload_utils.h"

void upload_to_CF(const std::string &file_path, cv::Mat &img) {
    CURL *curl;
    CURLcode res;
    struct curl_slist *headers = NULL;
    std::string url = "https://patient-butterfly-bfc7.haze.workers.dev/" + file_path;
    curl_global_init(CURL_GLOBAL_DEFAULT);

    curl = curl_easy_init();
    if(curl) {
        // 设置URL
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

        // 设置PUT请求
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);

        // 设置要上传的数据
        curl_easy_setopt(curl, CURLOPT_READDATA, img.data);
        curl_easy_setopt(curl, CURLOPT_INFILESIZE_LARGE, (curl_off_t)img.size().width * img.size().height * img.channels());

        // 添加header
        headers = curl_slist_append(headers, "Content-Type: application/octet-stream");
        headers = curl_slist_append(headers, "X-Custom-Auth-Key: Now1t1sShowtIME");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // 执行请求
        res = curl_easy_perform(curl);

        // 检查错误
        if(res != CURLE_OK)
            spdlog::error("curl_easy_perform() failed: {}", curl_easy_strerror(res));

        // 清理
        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();
}