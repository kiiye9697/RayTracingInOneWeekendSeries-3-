#ifndef HITTABLE_H
#define HITTABLE_H

#include "rtweekend.h"
#include"AABB.h"
//抽象虚基类，抽象出一个接口， 记录好光线和物体相交的逻辑，对应的相交法线、交点以及光线的t位置记录。
class material;//预先声明提供接口

class hit_record {
public:
    point3 p;
    vec3 normal;
    shared_ptr<material> mat;
    double t;
    bool front_face;

    double u;
    double v;

    //COMMENT：
    //在着色阶段再处理法向量也是可以解决的。但如果材质过多我们需要逐个材质进行向量重处理。
    //由于我们的材质与几何相比材质种类更多，因此我们考虑在此处的几何相交的部分就直接确认好法向量的方向。
    void set_face_normal(const ray& r, const vec3& outward_normal) {
        // Sets the hit record normal vector.
        // NOTE: the parameter `outward_normal` is assumed to have unit length.（我们假定这个向量是单位向量）

        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }
};



class hittable {
public:
    virtual ~hittable() = default;
    virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const = 0;
    virtual aabb bounding_box() const = 0;
};

//COMMENT:为了防止光线的自相交等情况，我们选择将对应的光线有计划的“偏移”一定数量来判断相交的逻辑；
// 然后在判断后，我们有意识地将这个偏移量还原，保证光线本身的正确性。
class translate : public hittable {
public:
    translate(shared_ptr<hittable> object, const vec3& offset)
        : object(object), offset(offset)
    {
        bbox = object->bounding_box() + offset;
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        // Move the ray backwards by the offset
        ray offset_r(r.origin() - offset, r.direction(), r.time());

        // Determine whether an intersection exists along the offset ray (and if so, where)
        if (!object->hit(offset_r, ray_t, rec))
            return false;

        // Move the intersection point forwards by the offset
        rec.p += offset;

        return true;
    }

    aabb bounding_box() const override { return bbox; }

private:
    shared_ptr<hittable> object;
    vec3 offset;
    aabb bbox;
};
class rotate_y : public hittable {
public:
    rotate_y(shared_ptr<hittable> object, double angle) : object(object) {
        auto radians = degrees_to_radians(angle);
        sin_theta = std::sin(radians);
        cos_theta = std::cos(radians);
        bbox = object->bounding_box();

        point3 min(infinity, infinity, infinity);
        point3 max(-infinity, -infinity, -infinity);

        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    auto x = i * bbox.x.max + (1 - i) * bbox.x.min;
                    auto y = j * bbox.y.max + (1 - j) * bbox.y.min;
                    auto z = k * bbox.z.max + (1 - k) * bbox.z.min;

                    auto newx = cos_theta * x + sin_theta * z;
                    auto newz = -sin_theta * x + cos_theta * z;

                    vec3 tester(newx, y, newz);

                    for (int c = 0; c < 3; c++) {
                        min[c] = std::fmin(min[c], tester[c]);
                        max[c] = std::fmax(max[c], tester[c]);
                    }
                }
            }
        }

        bbox = aabb(min, max);
    }
    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {

        // Transform the ray from world space to object space.

        auto origin = point3(
            (cos_theta * r.origin().x()) - (sin_theta * r.origin().z()),
            r.origin().y(),
            (sin_theta * r.origin().x()) + (cos_theta * r.origin().z())
        );

        auto direction = vec3(
            (cos_theta * r.direction().x()) - (sin_theta * r.direction().z()),
            r.direction().y(),
            (sin_theta * r.direction().x()) + (cos_theta * r.direction().z())
        );

        ray rotated_r(origin, direction, r.time());

        // Determine whether an intersection exists in object space (and if so, where).

        if (!object->hit(rotated_r, ray_t, rec))
            return false;

        // Transform the intersection from object space back to world space.

        rec.p = point3(
            (cos_theta * rec.p.x()) + (sin_theta * rec.p.z()),
            rec.p.y(),
            (-sin_theta * rec.p.x()) + (cos_theta * rec.p.z())
        );

        rec.normal = vec3(
            (cos_theta * rec.normal.x()) + (sin_theta * rec.normal.z()),
            rec.normal.y(),
            (-sin_theta * rec.normal.x()) + (cos_theta * rec.normal.z())
        );

        return true;

    }
    aabb bounding_box() const override { return bbox; }

private:
    shared_ptr<hittable> object;
    double sin_theta;
    double cos_theta;
    aabb bbox;
};
#endif

