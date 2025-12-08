idf_component_register(
    SRCS "ld2410c.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_uart esp_timer
)

# Add comprehensive compiler warnings for code quality
target_compile_options(${COMPONENT_LIB} PRIVATE
    # Standard warnings
    -Wall              # Enable common warnings
    -Wextra            # Enable extra warnings
    -Wconversion       # Warn on implicit type conversions
    -Wshadow           # Warn on variable shadowing
    -Wunused           # Warn on unused variables/functions
    -Wuninitialized    # Warn on uninitialized variables
    -Wstrict-prototypes      # Require C function prototypes
    -Wmissing-prototypes     # Warn on missing function prototypes
    -Wimplicit-fallthrough   # Warn on switch fallthrough

    # Format string security
    -Wformat=2         # Strict format string checking
    -Wformat-security  # Warn on format string vulnerabilities
    -Werror=format-security  # Make format security issues errors

    # Security hardening
    -fstack-protector-strong  # Stack smashing protection
    -D_FORTIFY_SOURCE=2       # Buffer overflow detection
)

# Add security hardening linker flags
target_link_options(${COMPONENT_LIB} PRIVATE
    -Wl,-z,relro      # Read-only relocations
    -Wl,-z,now        # Immediate binding
)
