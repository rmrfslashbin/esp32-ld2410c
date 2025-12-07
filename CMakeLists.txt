idf_component_register(
    SRCS "ld2410c.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)

# Add comprehensive compiler warnings for code quality
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wall              # Enable common warnings
    -Wextra            # Enable extra warnings
    -Wconversion       # Warn on implicit type conversions
    -Wshadow           # Warn on variable shadowing
    -Wformat=2         # Strict format string checking
    -Wformat-security  # Warn on format string vulnerabilities
    -Werror=format-security  # Make format security issues errors
    -Wimplicit-fallthrough   # Warn on switch fallthrough
    -Wunused           # Warn on unused variables/functions
    -Wuninitialized    # Warn on uninitialized variables
    -Wstrict-prototypes      # Require C function prototypes
    -Wmissing-prototypes     # Warn on missing function prototypes
)
