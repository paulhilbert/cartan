template <typename Scalar>
struct eigen_color_channel_max;

template <>
struct eigen_color_channel_max<float> {
    static float value() { return 1.f; }
};

template <>
struct eigen_color_channel_max<double> {
    static double value() { return 1.0; }
};

template <>
struct eigen_color_channel_max<uint8_t> {
    static uint8_t value() { return 255; }
};

template <>
struct eigen_color_channel_max<int8_t> {
    static int8_t value() { return 127; }
};

template <typename ScalarIn>
inline float
eigen_color_to_01(ScalarIn value) {
    return static_cast<float>(value) / static_cast<float>(eigen_color_channel_max<ScalarIn>::value());
}

template <>
inline float
eigen_color_to_01<float>(float value) {
    return value;
}


template <>
inline float
eigen_color_to_01<double>(double value) {
    return static_cast<float>(value);
}

template <typename ScalarOut>
inline ScalarOut
eigen_color_from_01(float normalized) {
    float value_max = static_cast<float>(eigen_color_channel_max<ScalarOut>::value());
    return static_cast<ScalarOut>(normalized * value_max);
}

template <>
inline float
eigen_color_from_01<float>(float normalized) {
    return normalized;
}

template <>
inline double
eigen_color_from_01<double>(float normalized) {
    return static_cast<double>(normalized);
}

template <typename ScalarOut, typename ScalarIn>
inline ScalarOut
eigen_color_channel_cast(ScalarIn in) {
    float inter = eigen_color_to_01<ScalarIn>(in);
    ScalarOut casted = eigen_color_from_01<ScalarOut>(inter);
    return casted;
}

template <typename ScalarOut, typename ScalarIn, int DimOut, int DimIn>
Eigen::Matrix<ScalarOut, DimOut, 1>
inline eigen_color_cast(const Eigen::Matrix<ScalarIn, DimIn, 1>& in) {
    Eigen::Matrix<ScalarOut, DimIn, 1> same_size;
    for (uint32_t i = 0; i < DimIn; ++i) {
        ScalarOut casted = eigen_color_channel_cast<ScalarOut, ScalarIn>(in[i]);
        same_size[i] = casted;
    }
    if (DimOut <= DimIn) return same_size.head(DimOut);
    Eigen::Matrix<ScalarOut, DimOut, 1> result;
    for (uint32_t i = 0; i < DimOut; ++i) {
        if (i < DimIn) {
            result[i] = same_size[i];
        } else {
            result[i] = eigen_color_channel_max<ScalarOut>::value();
        }
    }
    return result;
}
