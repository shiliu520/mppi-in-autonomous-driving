#pragma once

#include <string>
#include <vector>

enum class TrafficSignTypes;

/**
 * Class representing a CommonRoad traffic sign element.
 */
class TrafficSignElement {
  public:
    /**
     * Constructor of traffic sign element.
     *
     * @param trafficSignType Type of traffic sign element. Note this ID corresponds to the official national traffic
     * sign ID of a country.
     * @param values List of additional values, e.g., concrete speed limit.
     */
    explicit TrafficSignElement(TrafficSignTypes trafficSignType, std::vector<std::string> values = {});

    /**
     * Default Constructor without parameters for a traffic sign element.
     *
     */
    TrafficSignElement() = default;

    /**
     * Copy constructor of traffic sign element.
     *
     */
    TrafficSignElement(const TrafficSignElement &) = default;

    /**
     * Getter for ID of traffic sign element.
     *
     * @return ID of traffic sign element.
     */
    [[nodiscard]] TrafficSignTypes getTrafficSignType() const;

    /**
     * Adds ID of traffic sign element.
     *
     */
    void setTrafficSignType(TrafficSignTypes trafficSignType);

    /**
     * Getter for additional values of traffic sign element.
     *
     * @return Additional value.
     */
    [[nodiscard]] std::vector<std::string> getAdditionalValues() const;

    /**
     * Adds an additional value to a traffic sign element.
     *
     * @return Additional value.
     */
    void addAdditionalValue(const std::string &value);

    /**
     * Adds a list of additional values to a traffic sign element.
     *
     * @return Additional value.
     */
    void setAdditionalValues(const std::vector<std::string> &values);

    /**
     * Converts German traffic sign ID to string.
     *
     * @param signId Traffic sign ID from corresponding enum.
     * @return String which corresponds to traffic sign ID.
     */
    static std::string convertGermanTrafficSignIdToString(TrafficSignTypes signId);

  private:
    TrafficSignTypes trafficSignType; //**< Type of traffic sign, e.g., stop, yield. */
    std::vector<std::string>
        additionalValues; //**< list of additional values classifying traffic sign, e.g., velocity, weight, time */
};
