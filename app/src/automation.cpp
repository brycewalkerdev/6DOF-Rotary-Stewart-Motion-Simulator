#include "automation.h"

#include <cstdlib>
#include <cstring>

bool IsDocumentationMode() {
#if defined(STEWART_DOCUMENTATION_DYNAMIC)
    const char* value = std::getenv("STEWART_DOCUMENTATION_MODE");
    return value && (!std::strcmp(value, "1") || !std::strcmp(value, "true"));
#else
    static const bool enabled = [] {
        const char* value = std::getenv("STEWART_DOCUMENTATION_MODE");
        return value && (!std::strcmp(value, "1") || !std::strcmp(value, "true"));
    }();
    return enabled;
#endif
}
