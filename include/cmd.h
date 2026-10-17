#pragma once
#include <string_view>
#include <cstddef>
#include <charconv>
#include <system_error>

namespace cmd {
    namespace detail {
        constexpr bool is_space(const char c) noexcept {
            return c == ' ' || c == '\t';
        }

        inline bool parse_ints(std::string_view args, int* values, std::size_t count) noexcept {
            const char* current = args.data();
            const char* const end = current + args.size();

            for (std::size_t i = 0; i < count; ++i) {
                while (current != end && is_space(*current)) { ++current; }     // ignore ws
                if (current == end) return false;

                const auto [ptr, ec] = std::from_chars(current, end, values[i]);    // converts to int
                if (ec != std::errc{} || ptr == current) return false;

                current = ptr;

                if (current != end && !is_space(*current)) return false;    // space or end
            }

            while (current != end && is_space(*current)) { ++current; }     // excess ws
            return current == end;
        }
    }

    template<typename... Args>
    bool to_int(std::string_view args, Args&... variables) noexcept {
        constexpr std::size_t N = sizeof...(Args);

        int temp[N]; // Buffer en el stack (válido y de costo cero en runtime)
        if (!detail::parse_ints(args, temp, N)) {
            return false;
        }

        std::size_t i = 0;
        ((variables = temp[i++]), ...);

        return true;
    }
}
