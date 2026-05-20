#include <array>
#include <cstdint>
#include <utility>

class Channel {
public:
    Channel(uint32_t threshold) : threshold{threshold} {}
    uint32_t threshold;
};

class Ic {
public:
    static constexpr uint8_t CHANNEL_COUNT{4};
    
    Ic(std::array<uint32_t, CHANNEL_COUNT> thresholds) 
        : Ic(thresholds, std::make_index_sequence<CHANNEL_COUNT>{}) {}

private:
    template <std::size_t... Is>
    Ic(const std::array<uint32_t, CHANNEL_COUNT>& thresholds, std::index_sequence<Is...>)
        : channels{thresholds[Is]...} {}

public:
    std::array<Channel, CHANNEL_COUNT> channels;
};

class Pdu {
public:
    static constexpr uint8_t IC_COUNT{4};
    
    Pdu(std::array<std::array<uint32_t, Ic::CHANNEL_COUNT>, IC_COUNT> thresholds)
        : Pdu(thresholds, std::make_index_sequence<IC_COUNT>{}) {}

private:
    template <std::size_t... Is>
    Pdu(const std::array<std::array<uint32_t, Ic::CHANNEL_COUNT>, IC_COUNT>& thresholds, std::index_sequence<Is...>)
        : ics{thresholds[Is]...} {}

public:
    std::array<Ic, IC_COUNT> ics;
};

int main() {
    std::array<std::array<uint32_t, 4>, 4> th{};
    Pdu pdu(th);
    return 0;
}
