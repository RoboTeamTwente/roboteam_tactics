// included in utils.h

namespace rtt {
template<typename T, typename Container>
boost::optional<T> get_rand_element(const Container& cont) {
    if (cont.size() == 0) {
        return boost::optional<T>();
    }
    auto it = cont.begin();
    int idx = get_rand_int(cont.size());
    for (int i = 0; i < idx; i++, it++);
    return boost::optional<T>(*it);
}

template<typename T, typename Sequence>
boost::optional<T> get_rand_element_seq(const Sequence& vec) {
    if (vec.size() == 0) {
        return boost::optional<T>();
    }
    return boost::optional<T>(vec.at(get_rand_int(vec.size())));
}

template<typename T, typename Sequence>
bool sequence_contains(const Sequence& seq, T element) {
    return std::find(seq.begin(), seq.end(), element) != seq.end();
}

}