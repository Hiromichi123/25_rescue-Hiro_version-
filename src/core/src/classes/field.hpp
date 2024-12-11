#ifndef CLASSES_FIELD_HPP
#define CLASSES_FIELD_HPP

#include <memory>

class field{
public:
    float edge = 2.40; // 救援场地边长
    float start_zone_radius = 0.35; // 出发区半径
    float Compensation = 0.15; // 出发补偿0.15m
    std::unique_ptr<class safe_zone> safe_zone_ptr;
    std::unique_ptr<class exclusion_zone> exclusion_zone_ptr;

    field(){
        safe_zone_ptr = std::make_unique<safe_zone>(Compensation);
        exclusion_zone_ptr = std::make_unique<exclusion_zone>(Compensation);
    }

    // 对于红/蓝情况，己方安全区都在右侧，全局坐标区域固定
    class safe_zone{
    public:
        float Compensation;
        float x1, y1, x2, y2;

        safe_zone(float Compensation):Compensation(Compensation){
            x1 = 0.90 + Compensation;
            y1 = -0.90;
            x2 = 1.50 + Compensation;
            y2 = -1.20;
        }
    };

    // 对方安全区
    class exclusion_zone{
    public:
        float Compensation;
        float x1, y1, x2, y2;

        exclusion_zone(float Compensation):Compensation(Compensation){
            x1 = 0.90 + Compensation;
            y1 = 0.90;
            x2 = 1.50 + Compensation;
            y2 = 1.20;
        }
    };
};

#endif
