//
// Created by qqice on 24-6-3.
//

#include "upload_utils.h"

std::FILE* cvMatToFILE(const std::string& ext,const cv::Mat& img) {
    // Encode the cv::Mat object to a memory stream
    std::vector<uchar> buffer;
    cv::imencode(ext, img, buffer);
    std::string str(buffer.begin(), buffer.end());

    // Write the data to a std::FILE
    std::FILE* file = std::tmpfile();
    std::fwrite(str.data(), 1, str.size(), file);

    // Reset the file pointer to the beginning of the file
    std::rewind(file);

    return file;
}

void upload_to_CF(const std::string &file_path,const cv::Mat &img) {
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

        std::FILE* img_file = cvMatToFILE(file_path.substr(file_path.find_last_of('.')),img);

        // 设置要上传的数据
        curl_easy_setopt(curl, CURLOPT_READDATA, img_file);

        // 添加header
        headers = curl_slist_append(headers, "Content-Type: application/octet-stream");
        headers = curl_slist_append(headers, "X-Custom-Auth-Key: Now1t1sShowtIME");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

        // 执行请求
        res = curl_easy_perform(curl);

        // 检查错误
        if(res != CURLE_OK)
            spdlog::error("curl_easy_perform() failed: {}", curl_easy_strerror(res));
        std::fclose(img_file);
        // 清理
        curl_easy_cleanup(curl);
    }

    curl_global_cleanup();
}