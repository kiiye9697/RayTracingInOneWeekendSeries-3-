#ifndef HITTABLE_LIST_H
#define HITTABLE_LIST_H
#include "aabb.h"
#include "rtweekend.h"
#include "hittable_list.h"
#include<vector>

//shared_ptr属于全新的特性
class hittable_list : public hittable
{
public:
    std::vector<shared_ptr<hittable>> objects;

    hittable_list() {}
    hittable_list(shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }

    void add(shared_ptr<hittable> object) {
        objects.push_back(object);
        bbox = aabb(bbox, object->bounding_box());
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        hit_record temp_rec;
        bool hit_anything = false;
        auto closest_so_far = ray_t.max;
        //COMMENT:
        //光线击打到物体之后记录对应最近深度和对应的光线t，并且创建范围函数记录光线相交范围。
        for (const auto& object : objects) {
            if (object->hit(r, interval(ray_t.min, closest_so_far), temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }

        return hit_anything;
    }
    aabb bounding_box() const override { return bbox; }

private:
    aabb bbox;
};
#endif