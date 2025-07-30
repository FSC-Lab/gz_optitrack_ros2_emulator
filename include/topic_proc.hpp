#include <string>
#include <vector>

// convert the list of the input arguments as 
std::vector<std::string> GetSting(int argc, char *argv[])
{
    std::vector<std::string> res;
    auto size = static_cast<uint32_t>(argc);
    res.reserve(size);
    for (uint32_t i = 0; i < size; i++) {
        res.emplace_back(argv[i]);
    }
    return res;
}