#pragma once
#include "phys2d/body.hpp"
#include "phys2d/collision.hpp"
#include "phys2d/joint.hpp"
#include "phys2d/query.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace phys2d {

enum class BroadPhaseType : unsigned char { SpatialHash, SAP };

class World {
public:
    using BroadPhaseType = phys2d::BroadPhaseType;
    using Callback = std::function<void(const Manifold&)>;

    struct BodyIterator {
        std::vector<std::unique_ptr<Body>>::iterator it{};
        BodyIterator() = default;
        explicit BodyIterator(std::vector<std::unique_ptr<Body>>::iterator iter) noexcept
            : it(iter) {
        }
        BodyIterator& operator++() noexcept {
            ++it;
            return *this;
        }
        BodyIterator operator++(int) noexcept {
            BodyIterator tmp = *this;
            ++it;
            return tmp;
        }
        Body& operator*() const noexcept {
            return **it;
        }
        Body* operator->() const noexcept {
            return it->get();
        }
        bool operator==(BodyIterator const& other) const noexcept {
            return it == other.it;
        }
        bool operator!=(BodyIterator const& other) const noexcept {
            return it != other.it;
        }
    };

    struct BodyConstIterator {
        std::vector<std::unique_ptr<Body>>::const_iterator it{};
        BodyConstIterator() = default;
        explicit BodyConstIterator(std::vector<std::unique_ptr<Body>>::const_iterator iter) noexcept
            : it(iter) {
        }
        BodyConstIterator& operator++() noexcept {
            ++it;
            return *this;
        }
        BodyConstIterator operator++(int) noexcept {
            BodyConstIterator tmp = *this;
            ++it;
            return tmp;
        }
        Body const& operator*() const noexcept {
            return **it;
        }
        Body const* operator->() const noexcept {
            return it->get();
        }
        bool operator==(BodyConstIterator const& other) const noexcept {
            return it == other.it;
        }
        bool operator!=(BodyConstIterator const& other) const noexcept {
            return it != other.it;
        }
    };

    Vec2 gravity{0.f, 900.f};
    int velocityIterations = 8;
    int positionIterations = 3;
    int substeps = 1;
    float cellSize = 64.f;
    BroadPhaseType broadPhaseType = BroadPhaseType::SpatialHash;
    // Ruido residual de un cuerpo en reposo en este solver ≈ 0.5 px/s;
    // el umbral por defecto está holgadamente por encima (1 px/s ≈ 1 px/s
    // de deriva, perceptualmente quieto).
    float sleepVelocity = 1.0f;
    float sleepTime = 0.5f;
    bool bullet = false;

    World() = default;

    // Copiar un World clona cuerpos y joints (Joint::clone()).
    World(World const& other)
        : gravity(other.gravity), velocityIterations(other.velocityIterations),
          positionIterations(other.positionIterations), substeps(other.substeps),
          cellSize(other.cellSize), broadPhaseType(other.broadPhaseType),
          sleepVelocity(other.sleepVelocity), sleepTime(other.sleepTime), bullet(other.bullet),
          contacts_(other.contacts_), prevContacts_(other.prevContacts_),
          prevGeom_(other.prevGeom_), curGeom_(other.curGeom_), gridEntries_(other.gridEntries_),
          warmEntries_(other.warmEntries_), beginCallbacks_(other.beginCallbacks_),
          endCallbacks_(other.endCallbacks_) {
        bodies_.reserve(other.bodies_.size());
        ptrs_.reserve(other.bodies_.size());
        cptrs_.reserve(other.bodies_.size());
        for (auto const& bp : other.bodies_) {
            bodies_.push_back(std::make_unique<Body>(*bp));
            ptrs_.push_back(bodies_.back().get());
            cptrs_.push_back(bodies_.back().get());
        }
        joints_.reserve(other.joints_.size());
        for (auto const& jointPtr : other.joints_) {
            joints_.push_back(jointPtr->clone());
        }
    }

    World(World&& other) noexcept
        : gravity(other.gravity), velocityIterations(other.velocityIterations),
          positionIterations(other.positionIterations), substeps(other.substeps),
          cellSize(other.cellSize), broadPhaseType(other.broadPhaseType),
          sleepVelocity(other.sleepVelocity), sleepTime(other.sleepTime), bullet(other.bullet),
          bodies_(std::move(other.bodies_)), ptrs_(std::move(other.ptrs_)),
          cptrs_(std::move(other.cptrs_)), joints_(std::move(other.joints_)),
          contacts_(std::move(other.contacts_)), prevContacts_(std::move(other.prevContacts_)),
          prevGeom_(std::move(other.prevGeom_)), curGeom_(std::move(other.curGeom_)),
          gridEntries_(std::move(other.gridEntries_)), warmEntries_(std::move(other.warmEntries_)),
          beginCallbacks_(std::move(other.beginCallbacks_)),
          endCallbacks_(std::move(other.endCallbacks_)) {
    }

    World& operator=(World const& other) {
        if (this == &other) {
            return *this;
        }
        gravity = other.gravity;
        velocityIterations = other.velocityIterations;
        positionIterations = other.positionIterations;
        substeps = other.substeps;
        cellSize = other.cellSize;
        broadPhaseType = other.broadPhaseType;
        sleepVelocity = other.sleepVelocity;
        sleepTime = other.sleepTime;
        bullet = other.bullet;
        bodies_.clear();
        ptrs_.clear();
        cptrs_.clear();
        joints_.clear();
        bodies_.reserve(other.bodies_.size());
        ptrs_.reserve(other.bodies_.size());
        cptrs_.reserve(other.bodies_.size());
        for (auto const& bp : other.bodies_) {
            bodies_.push_back(std::make_unique<Body>(*bp));
            ptrs_.push_back(bodies_.back().get());
            cptrs_.push_back(bodies_.back().get());
        }
        joints_.reserve(other.joints_.size());
        for (auto const& jointPtr : other.joints_) {
            joints_.push_back(jointPtr->clone());
        }
        contacts_ = other.contacts_;
        prevContacts_ = other.prevContacts_;
        prevGeom_ = other.prevGeom_;
        curGeom_ = other.curGeom_;
        gridEntries_ = other.gridEntries_;
        warmEntries_ = other.warmEntries_;
        beginCallbacks_ = other.beginCallbacks_;
        endCallbacks_ = other.endCallbacks_;
        return *this;
    }

    World& operator=(World&& other) noexcept {
        if (this == &other) {
            return *this;
        }
        gravity = other.gravity;
        velocityIterations = other.velocityIterations;
        positionIterations = other.positionIterations;
        substeps = other.substeps;
        cellSize = other.cellSize;
        broadPhaseType = other.broadPhaseType;
        sleepVelocity = other.sleepVelocity;
        sleepTime = other.sleepTime;
        bullet = other.bullet;
        joints_ = std::move(other.joints_);
        bodies_ = std::move(other.bodies_);
        // Importante: otros ptrs_/cptrs_ no se mueven (no son propietarios),
        // así que el mundo de origen queda vacío para no dejar punteros crudos.
        other.ptrs_.clear();
        other.cptrs_.clear();
        ptrs_.clear();
        cptrs_.clear();
        ptrs_.reserve(bodies_.size());
        cptrs_.reserve(bodies_.size());
        for (auto& bp : bodies_) {
            ptrs_.push_back(bp.get());
            cptrs_.push_back(bp.get());
        }
        contacts_ = std::move(other.contacts_);
        prevContacts_ = std::move(other.prevContacts_);
        prevGeom_ = std::move(other.prevGeom_);
        curGeom_ = std::move(other.curGeom_);
        gridEntries_ = std::move(other.gridEntries_);
        warmEntries_ = std::move(other.warmEntries_);
        beginCallbacks_ = std::move(other.beginCallbacks_);
        endCallbacks_ = std::move(other.endCallbacks_);
        return *this;
    }

    ~World() = default;

    /// Span de solo lectura sobre cuerpos crudos (queries, drawWorld, raycastPoint).
    /// Para mutar, usar body(id) o la iteración de World.
    [[nodiscard]] std::span<const Body*> bodiesSpan() const noexcept {
        return std::span<const Body*>(cptrs_.data(), cptrs_.size());
    }

    /// Acceso por id: lanza std::out_of_range si el id no existe.
    [[nodiscard]] Body& body(int id) {
        if (id < 0 || static_cast<std::size_t>(id) >= bodies_.size()) {
            throw std::out_of_range{"phys2d::World::body: id fuera de rango"};
        }
        return *bodies_[static_cast<std::size_t>(id)];
    }

    [[nodiscard]] Body const& body(int id) const {
        if (id < 0 || static_cast<std::size_t>(id) >= bodies_.size()) {
            throw std::out_of_range{"phys2d::World::body: id fuera de rango"};
        }
        return *bodies_[static_cast<std::size_t>(id)];
    }

    /// Número de joints vivos.
    [[nodiscard]] std::size_t jointCount() const noexcept {
        return joints_.size();
    }

    /// Acceso por índice: lanza std::out_of_range si el índice no existe.
    [[nodiscard]] Joint& joint(int id) {
        Joint* result = jointPtr(id);
        if (result == nullptr) {
            throw std::out_of_range{"phys2d::World::joint: índice fuera de rango"};
        }
        return *result;
    }
    [[nodiscard]] Joint const& joint(int id) const {
        Joint const* result = jointPtr(id);
        if (result == nullptr) {
            throw std::out_of_range{"phys2d::World::joint: índice fuera de rango"};
        }
        return *result;
    }

    /// Añade un joint ya construido (p. ej. std::make_unique<Distance>(a, b, rest)).
    /// Lanza std::out_of_range si a/b no son ids válidos de cuerpos o std::invalid_argument si a ==
    /// b.
    int addJoint(std::unique_ptr<Joint> joint) {
        if (!joint) {
            throw std::invalid_argument{"phys2d::World::addJoint: joint nulo"};
        }
        std::size_t const count = bodies_.size();
        auto const validIndex = [count](int id) {
            return id >= 0 && static_cast<std::size_t>(id) < count;
        };
        if (!validIndex(joint->a) || !validIndex(joint->b)) {
            throw std::out_of_range{"phys2d::World::addJoint: id de cuerpo fuera de rango"};
        }
        if (joint->a == joint->b) {
            throw std::invalid_argument{"phys2d::World::addJoint: a y b deben ser distintos"};
        }
        joints_.push_back(std::move(joint));
        return static_cast<int>(joints_.size()) - 1;
    }

    /// Ultra fácil: w.addJoint<Distance>(a, b, 150.f);
    template <typename JointT, typename... Args> int addJoint(Args&&... args) {
        return addJoint(std::make_unique<JointT>(std::forward<Args>(args)...));
    }

    // Helpers con parámetros Vec2 concretos: aceptan {0, 0} a pelo, cosa que
    // la deducción del template variádico no puede hacer con braced-init.
    int addDistanceJoint(int a, int b, float rest, Vec2 anchorA = {}, Vec2 anchorB = {}) {
        return addJoint<Distance>(a, b, rest, anchorA, anchorB);
    }

    int addRevoluteJoint(int a, int b, Vec2 anchorA, Vec2 anchorB) {
        return addJoint<Revolute>(a, b, anchorA, anchorB);
    }

    int addWeldJoint(int a, int b, Vec2 anchorA, Vec2 anchorB) {
        return addJoint<Weld>(a, b, anchorA, anchorB);
    }

    int addPrismaticJoint(int a, int b, Vec2 axis, float lower, float upper, Vec2 anchorA = {},
                          Vec2 anchorB = {}) {
        auto joint = std::make_unique<Prismatic>(a, b, axis, lower, upper);
        joint->anchorA = anchorA;
        joint->anchorB = anchorB;
        return addJoint(std::move(joint));
    }

    int addSliderJoint(int a, int b, Vec2 axis, float lower, float upper, Vec2 anchorA = {},
                       Vec2 anchorB = {}) {
        auto joint = std::make_unique<Slider>(a, b, axis, lower, upper);
        joint->anchorA = anchorA;
        joint->anchorB = anchorB;
        return addJoint(std::move(joint));
    }

    int addPulleyJoint(int a, int b, Vec2 anchorA, Vec2 anchorB, Vec2 groundA, Vec2 groundB,
                       float ratio) {
        return addJoint<Pulley>(a, b, anchorA, anchorB, groundA, groundB, ratio);
    }

    int addSpringJoint(int a, int b, float rest, float stiffness, float damping, Vec2 anchorA = {},
                       Vec2 anchorB = {}) {
        auto joint = std::make_unique<Spring>(a, b, rest, stiffness, damping);
        joint->anchorA = anchorA;
        joint->anchorB = anchorB;
        return addJoint(std::move(joint));
    }

    int addRopeJoint(int a, int b, float maxLength) {
        return addJoint<Rope>(a, b, maxLength);
    }

    int addGearJoint(int a, int b, float ratio) {
        return addJoint<Gear>(a, b, ratio);
    }

    [[nodiscard]] BodyIterator begin() noexcept {
        return BodyIterator{bodies_.begin()};
    }
    [[nodiscard]] BodyIterator end() noexcept {
        return BodyIterator{bodies_.end()};
    }
    [[nodiscard]] BodyConstIterator begin() const noexcept {
        return BodyConstIterator{bodies_.begin()};
    }
    [[nodiscard]] BodyConstIterator end() const noexcept {
        return BodyConstIterator{bodies_.end()};
    }
    [[nodiscard]] BodyConstIterator cbegin() const noexcept {
        return BodyConstIterator{bodies_.begin()};
    }
    [[nodiscard]] BodyConstIterator cend() const noexcept {
        return BodyConstIterator{bodies_.end()};
    }

    [[nodiscard]] std::size_t size() const noexcept {
        return bodies_.size();
    }
    [[nodiscard]] bool empty() const noexcept {
        return bodies_.empty();
    }

    void clear() noexcept {
        bodies_.clear();
        ptrs_.clear();
        cptrs_.clear();
        joints_.clear();
        contacts_.clear();
        prevContacts_.clear();
        prevGeom_.clear();
        curGeom_.clear();
        preForceVel_.clear();
        gridEntries_.clear();
        warmEntries_.clear();
        pairs_.clear();
        radii_.clear();
        startPos_.clear();
        startVel_.clear();
        startAngVel_.clear();
        bestT_.clear();
        bestN_.clear();
        bestOther_.clear();
        hasHit_.clear();
        swept_.clear();
        solver_.clear();
        eps_.clear();
        active_.clear();
        prevKeys_.clear();
        curKeys_.clear();
        pairsAux_.clear();
        gridEntriesAux_.clear();
        radixCount_.fill(0U);
    }

    void setBroadPhase(BroadPhaseType bp) noexcept {
        broadPhaseType = bp;
    }
    void setBroadPhaseType(BroadPhaseType bp) noexcept {
        broadPhaseType = bp;
    }

    [[nodiscard]] int addBody(BodyParams p) {
        auto bref = std::make_unique<Body>();
        bref->configure(p);
        return registerBody(std::move(bref));
    }

    [[nodiscard]] int addBody(BodyParams p, Shape shapeValue) {
        auto bref = std::make_unique<Body>();
        bref->configure(p);
        bref->shape = shapeValue;
        bref->updateInertia();
        return registerBody(std::move(bref));
    }

    [[nodiscard]] int addCircle(Vec2 pos, float radius, Vec2 vel = {}, float mass = 1.f,
                                BodyType bodyType = BodyType::Dynamic) {
        CircleParams p{};
        p.pos = pos;
        p.radius = radius;
        p.vel = vel;
        p.mass = mass;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = bodyType;
        return addCircle(p);
    }

    [[nodiscard]] int addCircle(CircleParams p) {
        auto bref = std::make_unique<Body>();
        bref->configure(p);
        return registerBody(std::move(bref));
    }

    [[nodiscard]] int addBox(Vec2 pos, Vec2 half, Vec2 vel = {}, float mass = 1.f,
                             float angle = 0.f, BodyType bodyType = BodyType::Dynamic) {
        BoxParams p{};
        p.pos = pos;
        p.half = half;
        p.vel = vel;
        p.mass = mass;
        p.angle = angle;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = bodyType;
        return addBox(p);
    }

    [[nodiscard]] int addBox(BoxParams p) {
        auto bref = std::make_unique<Body>();
        bref->configure(p);
        return registerBody(std::move(bref));
    }

    [[nodiscard]] int addPolygon(Vec2 pos, std::vector<Vec2> vertices, Vec2 vel = {},
                                 float mass = 1.f, float angle = 0.f,
                                 BodyType bodyType = BodyType::Dynamic) {
        PolygonParams p{};
        p.pos = pos;
        p.verts = std::move(vertices);
        p.vel = vel;
        p.mass = mass;
        p.angle = angle;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = bodyType;
        return addPolygon(p);
    }

    [[nodiscard]] int addPolygon(PolygonParams p) {
        auto bref = std::make_unique<Body>();
        bref->configure(p);
        return registerBody(std::move(bref));
    }

    [[nodiscard]] int addCapsule(Vec2 pos, float radius, float length, Vec2 vel = {},
                                 float mass = 1.f, float angle = 0.f,
                                 BodyType bodyType = BodyType::Dynamic) {
        CapsuleParams p{};
        p.pos = pos;
        p.radius = radius;
        p.length = length;
        p.vel = vel;
        p.mass = mass;
        p.angle = angle;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = bodyType;
        return addCapsule(p);
    }

    [[nodiscard]] int addCapsule(CapsuleParams p) {
        auto bref = std::make_unique<Body>();
        bref->configure(p);
        return registerBody(std::move(bref));
    }

    [[nodiscard]] int addSegment(Vec2 pos, Vec2 a, Vec2 b, float radius = 0.f, Vec2 vel = {},
                                 float mass = 1.f, float angle = 0.f,
                                 BodyType bodyType = BodyType::Static) {
        SegmentParams p{};
        p.pos = pos;
        p.a = a;
        p.b = b;
        p.radius = radius;
        p.vel = vel;
        p.mass = mass;
        p.angle = angle;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = bodyType;
        return addSegment(p);
    }

    [[nodiscard]] int addSegment(SegmentParams p) {
        auto bref = std::make_unique<Body>();
        bref->configure(p);
        return registerBody(std::move(bref));
    }

    [[nodiscard]] int addStaticCircle(Vec2 pos, float radius) {
        CircleParams p{};
        p.pos = pos;
        p.radius = radius;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = BodyType::Static;
        return addCircle(p);
    }

    [[nodiscard]] int addStaticCircle(CircleParams p) {
        p.type = BodyType::Static;
        return addCircle(p);
    }

    [[nodiscard]] int addStaticBox(Vec2 pos, Vec2 half, float angle = 0.f) {
        BoxParams p{};
        p.pos = pos;
        p.half = half;
        p.angle = angle;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = BodyType::Static;
        return addBox(p);
    }

    [[nodiscard]] int addStaticBox(BoxParams p) {
        p.type = BodyType::Static;
        return addBox(p);
    }

    [[nodiscard]] int addStaticPolygon(PolygonParams p) {
        p.type = BodyType::Static;
        return addPolygon(p);
    }

    [[nodiscard]] int addStaticCapsule(CapsuleParams p) {
        p.type = BodyType::Static;
        return addCapsule(p);
    }

    [[nodiscard]] int addKinematicCircle(Vec2 pos, float radius, Vec2 vel = {}) {
        CircleParams p{};
        p.pos = pos;
        p.radius = radius;
        p.vel = vel;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = BodyType::Kinematic;
        return addCircle(p);
    }

    [[nodiscard]] int addKinematicCircle(CircleParams p) {
        p.type = BodyType::Kinematic;
        return addCircle(p);
    }

    [[nodiscard]] int addKinematicBox(Vec2 pos, Vec2 half, Vec2 vel = {}, float angle = 0.f) {
        BoxParams p{};
        p.pos = pos;
        p.half = half;
        p.vel = vel;
        p.angle = angle;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = BodyType::Kinematic;
        return addBox(p);
    }

    [[nodiscard]] int addKinematicBox(BoxParams p) {
        p.type = BodyType::Kinematic;
        return addBox(p);
    }

    [[nodiscard]] int addKinematicPolygon(PolygonParams p) {
        p.type = BodyType::Kinematic;
        return addPolygon(p);
    }

    [[nodiscard]] int addKinematicCapsule(CapsuleParams p) {
        p.type = BodyType::Kinematic;
        return addCapsule(p);
    }

    [[nodiscard]] int addDynamicCircle(Vec2 pos, float radius, Vec2 vel = {}, float mass = 1.f) {
        CircleParams p{};
        p.pos = pos;
        p.radius = radius;
        p.vel = vel;
        p.mass = mass;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = BodyType::Dynamic;
        return addCircle(p);
    }

    [[nodiscard]] int addDynamicCircle(CircleParams p) {
        p.type = BodyType::Dynamic;
        return addCircle(p);
    }

    [[nodiscard]] int addDynamicBox(Vec2 pos, Vec2 half, Vec2 vel = {}, float mass = 1.f,
                                    float angle = 0.f) {
        BoxParams p{};
        p.pos = pos;
        p.half = half;
        p.vel = vel;
        p.mass = mass;
        p.angle = angle;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = BodyType::Dynamic;
        return addBox(p);
    }

    [[nodiscard]] int addDynamicBox(BoxParams p) {
        p.type = BodyType::Dynamic;
        return addBox(p);
    }

    [[nodiscard]] int addDynamicPolygon(Vec2 pos, std::vector<Vec2> vertices, Vec2 vel = {},
                                        float mass = 1.f, float angle = 0.f) {
        PolygonParams p{};
        p.pos = pos;
        p.verts = std::move(vertices);
        p.vel = vel;
        p.mass = mass;
        p.angle = angle;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = BodyType::Dynamic;
        return addPolygon(p);
    }

    [[nodiscard]] int addDynamicPolygon(PolygonParams p) {
        p.type = BodyType::Dynamic;
        return addPolygon(p);
    }

    [[nodiscard]] int addDynamicCapsule(Vec2 pos, float radius, float length, Vec2 vel = {},
                                        float mass = 1.f, float angle = 0.f) {
        CapsuleParams p{};
        p.pos = pos;
        p.radius = radius;
        p.length = length;
        p.vel = vel;
        p.mass = mass;
        p.angle = angle;
        p.friction = 0.3f;
        p.restitution = 0.2f;
        p.linearDamping = 0.01f;
        p.angularDamping = 0.01f;
        p.type = BodyType::Dynamic;
        return addCapsule(p);
    }

    [[nodiscard]] int addDynamicCapsule(CapsuleParams p) {
        p.type = BodyType::Dynamic;
        return addCapsule(p);
    }

    [[nodiscard]] int addStaticSegment(SegmentParams p) {
        p.type = BodyType::Static;
        return addSegment(p);
    }

    [[nodiscard]] int addKinematicSegment(SegmentParams p) {
        p.type = BodyType::Kinematic;
        return addSegment(p);
    }

    [[nodiscard]] int addDynamicSegment(SegmentParams p) {
        p.type = BodyType::Dynamic;
        return addSegment(p);
    }

    [[nodiscard]] std::span<const Manifold> contacts() noexcept {
        return std::span<const Manifold>(contacts_.data(), contacts_.size());
    }
    [[nodiscard]] std::span<const Manifold> contacts() const noexcept {
        return std::span<const Manifold>(contacts_.data(), contacts_.size());
    }

    void onBeginContact(Callback cb) {
        if (cb) {
            beginCallbacks_.push_back(std::move(cb));
        }
    }

    void onEndContact(Callback cb) {
        if (cb) {
            endCallbacks_.push_back(std::move(cb));
        }
    }

    void applyForce(int id, Vec2 force) {
        body(id).applyForce(force);
    }

    /// Fuerza en un punto del mundo: produce también torque.
    void applyForce(int id, Vec2 force, Vec2 worldPoint) {
        body(id).applyForce(force, worldPoint);
    }

    void applyLinearImpulse(int id, Vec2 imp) {
        body(id).applyLinearImpulse(imp);
    }

    void applyAngularImpulse(int id, float imp) {
        body(id).applyAngularImpulse(imp);
    }

    void clearForces() noexcept {
        for (auto& bp : bodies_) {
            bp->clearForces();
        }
    }

    void step(float dt) {
        step(dt, substeps);
    }

    /// Lanza std::invalid_argument si dt no es finito y > 0. Un dt grande no se
    /// trunca en silencio: el adaptador de subpasos lo subdivide (tope 128).
    void step(float dt, int substepsArg) {
        if (!std::isfinite(dt) || !(dt > 0.f)) {
            throw std::invalid_argument{"phys2d::World::step: dt debe ser finito y > 0"};
        }
        int const base = substepsArg > 0 ? substepsArg : (substeps > 0 ? substeps : 1);
        float maxRatio = 0.f;
        for (auto const& bp : bodies_) {
            if (bp->isStatic() || bp->isSleeping) {
                continue;
            }
            float const r = bp->boundingRadius();
            if (!(r > 1e-6f)) {
                continue;
            }
            float const lin = bp->vel.length() * dt;
            float const av = bp->angVel >= 0.f ? bp->angVel : -bp->angVel;
            float const rot = av * r * dt;
            float const disp = lin > rot ? lin : rot;
            float const ratio = disp / r;
            if (ratio > maxRatio) {
                maxRatio = ratio;
            }
        }
        // Limitar el desplazamiento por substep a medio radio impide saltarse
        // paredes finas: el cuerpo nunca avanza más que su propio tamaño.
        int adaptive = 1;
        if (maxRatio > 0.5f) {
            float const needed = std::ceil(maxRatio / 0.5f);
            adaptive = needed >= 128.f ? 128 : static_cast<int>(needed);
        }
        int n = base > adaptive ? base : adaptive;
        if (n < 1) {
            n = 1;
        }
        if (n > 128) {
            n = 128;
        }
        float h = dt / static_cast<float>(n);
        // Instantánea para los callbacks: una sola notificación por step y
        // no por substep. Se parte del estado al finalizar el step anterior.
        bool const fireCallbacks = !beginCallbacks_.empty() || !endCallbacks_.empty();
        std::vector<Manifold> const startContacts =
            fireCallbacks ? prevContacts_ : std::vector<Manifold>{};
        for (int i = 0; i < n; ++i) {
            stepOnce(h);
        }
        if (fireCallbacks) {
            fireContactCallbacks(startContacts);
        }
        clearForces();
    }

    /// Ultra fácil: auto hit = w.raycast({400, 0}, {0, 1}, 600.f);
    [[nodiscard]] RayHit raycast(Vec2 origin, Vec2 dir, float maxDist) const {
        return raycastPoint(origin, dir, maxDist, bodiesSpan());
    }

    [[nodiscard]] bool raycast(Vec2 origin, Vec2 dir, float maxDist, Vec2& hitPoint,
                               int& hitBody) const {
        RayHit const hit = raycast(origin, dir, maxDist);
        hitPoint = hit.point;
        hitBody = hit.bodyId;
        return hit.hit;
    }

private:
    [[nodiscard]] Joint* jointPtr(int id) noexcept {
        if (id < 0 || static_cast<std::size_t>(id) >= joints_.size()) {
            return nullptr;
        }
        return joints_[static_cast<std::size_t>(id)].get();
    }
    [[nodiscard]] Joint const* jointPtr(int id) const noexcept {
        if (id < 0 || static_cast<std::size_t>(id) >= joints_.size()) {
            return nullptr;
        }
        return joints_[static_cast<std::size_t>(id)].get();
    }

    /// Un SolverContact por punto de contacto (1 o 2 por Manifold): así el
    /// impulso en cada punto genera el torque correcto sobre ambos cuerpos.
    struct SolverContact {
        int a = -1;
        int b = -1;
        int pointIndex = 0;
        int pointCount = 1;
        Vec2 point{};
        Vec2 normal{1.f, 0.f};
        Vec2 armA{};
        Vec2 armB{};
        float massNormal = 0.f;
        float massTangent = 0.f;
        float accumNormal = 0.f;
        float accumTangent = 0.f;
        float restitutionBias = 0.f;
        float friction = 0.f;
        float depth = 0.f;
    };

    struct WarmStart {
        float normal1 = 0.f;
        float tangent1 = 0.f;
        float normal2 = 0.f;
        float tangent2 = 0.f;
    };

    struct CellEntry {
        long long key = 0;
        int id = 0;
    };

    struct WarmEntry {
        long long key = 0;
        WarmStart ws{};
    };

    struct SAPEndpoint {
        float x = 0.f;
        int id = 0;
        bool isMax = false;
    };

    struct ManifoldGeom {
        Vec2 posA{};
        Vec2 posB{};
        float angA = 0.f;
        float angB = 0.f;
    };

    std::vector<std::unique_ptr<Body>> bodies_;
    std::vector<Body*> ptrs_;
    // Velocidades al principio del substep (antes de integrar fuerzas),
    // usadas para decidir si una pareja está en movimiento.
    std::vector<Vec2> preForceVel_;
    // Espejo const de ptrs_: hace falta para servir spans const sin const_cast.
    mutable std::vector<const Body*> cptrs_;
    std::vector<std::unique_ptr<Joint>> joints_;
    std::vector<Manifold> contacts_;
    std::vector<Manifold> prevContacts_;
    std::vector<ManifoldGeom> prevGeom_;
    std::vector<ManifoldGeom> curGeom_;
    std::vector<CellEntry> gridEntries_;
    std::vector<CellEntry> gridEntriesAux_;
    std::vector<WarmEntry> warmEntries_;
    std::vector<std::pair<int, int>> pairs_;
    std::vector<std::pair<int, int>> pairsAux_;
    std::array<std::size_t, 256> radixCount_{};
    std::vector<float> radii_;
    std::vector<Vec2> startPos_;
    std::vector<Vec2> startVel_;
    std::vector<float> startAngVel_;
    std::vector<float> bestT_;
    std::vector<Vec2> bestN_;
    std::vector<int> bestOther_;
    std::vector<char> hasHit_;
    std::vector<char> swept_;
    std::vector<SolverContact> solver_;
    std::vector<SAPEndpoint> eps_;
    std::vector<int> active_;
    std::unordered_set<long long> prevKeys_;
    std::unordered_set<long long> curKeys_;
    std::vector<Callback> beginCallbacks_;
    std::vector<Callback> endCallbacks_;

    /// Un cuerpo se considera "en movimiento" para despertar a su pareja si
    /// su velocidad PRE-GRAVEDAD (el estado liquido al acabar el substep
    /// anterior) supera sleepVelocity, o si su |angVel| lo supera. Consultar
    /// la velocidad ya con la gravedad integrada confunde un cuerpo en reposo
    /// (que nunca se mueve tras el solver) con uno cayendo.
    [[nodiscard]] bool partnerMovingPre(std::size_t idx) const noexcept {
        Body const& bref = *ptrs_[idx];
        if (!bref.isDynamic() || bref.isSleeping) {
            return false;
        }
        float const av = bref.angVel >= 0.f ? bref.angVel : -bref.angVel;
        return preForceVel_[idx].length() > sleepVelocity || av > sleepVelocity;
    }

    [[nodiscard]] static long long pairKey(int a, int b) noexcept {
        int lo = a < b ? a : b;
        int hi = a < b ? b : a;
        return (static_cast<long long>(lo) << 32) ^ static_cast<unsigned int>(hi);
    }

    [[nodiscard]] static long long hashCell(int cx, int cy) noexcept {
        return (static_cast<long long>(cx) << 32) ^ static_cast<unsigned int>(cy);
    }

    static void validateBody(Body const& bref) {
        // NaN/inf en cualquier campo contamina el mundo en silencio: se
        // rechaza todo lo no finito antes de registrar el cuerpo, igual que
        // los rangos físicos inválidos.
        auto const finite = [](float v) { return std::isfinite(v); };
        auto const finiteVec = [](Vec2 v) { return std::isfinite(v.x) && std::isfinite(v.y); };
        if (!finite(bref.pos.x) || !finite(bref.pos.y) || !finite(bref.vel.x) ||
            !finite(bref.vel.y) || !finite(bref.angle) || !finite(bref.angVel) ||
            !finite(bref.mass) || !finite(bref.friction) || !finite(bref.restitution) ||
            !finite(bref.linearDamping) || !finite(bref.angularDamping)) {
            throw std::invalid_argument{"phys2d: valores no finitos (NaN/inf)"};
        }
        if (!(bref.friction >= 0.f)) {
            throw std::invalid_argument{"phys2d: la fricción debe ser >= 0"};
        }
        if (!(bref.restitution >= 0.f) || bref.restitution > 1.f) {
            throw std::invalid_argument{"phys2d: la restitución debe estar en [0, 1]"};
        }
        if (!(bref.linearDamping >= 0.f) || !(bref.angularDamping >= 0.f)) {
            throw std::invalid_argument{"phys2d: los amortiguamientos deben ser >= 0"};
        }
        if (bref.isDynamic() && !(bref.mass > 0.f)) {
            throw std::invalid_argument{"phys2d: la masa dinámica debe ser > 0"};
        }
        bool const ok = std::visit(
            [&finite, &finiteVec](auto const& shape) {
                using T = std::decay_t<decltype(shape)>;
                if constexpr (std::is_same_v<T, Circle>) {
                    return finite(shape.radius) && shape.radius > 0.f;
                } else if constexpr (std::is_same_v<T, AABB>) {
                    return finiteVec(shape.lower) && finiteVec(shape.upper) &&
                           shape.width() > 0.f && shape.height() > 0.f;
                } else if constexpr (std::is_same_v<T, ConvexPolygon>) {
                    for (Vec2 const v : shape.verts) {
                        if (!finiteVec(v)) {
                            return false;
                        }
                    }
                    return shape.isValid();
                } else if constexpr (std::is_same_v<T, Capsule>) {
                    // dir debe ser finito y no nulo (configure() ya lo normaliza).
                    return finite(shape.radius) && shape.radius > 0.f && finite(shape.length) &&
                           shape.length > 0.f && finiteVec(shape.dir) &&
                           shape.dir.length2() > 1e-12f;
                } else {
                    return finiteVec(shape.a) && finiteVec(shape.b) && finite(shape.radius) &&
                           shape.radius >= 0.f && shape.length() > 0.f;
                }
            },
            bref.shape);
        if (!ok) {
            throw std::invalid_argument{"phys2d: forma degenerada o polígono no convexo"};
        }
    }

    [[nodiscard]] int registerBody(std::unique_ptr<Body> bref) {
        validateBody(*bref);
        bodies_.push_back(std::move(bref));
        ptrs_.push_back(bodies_.back().get());
        cptrs_.push_back(bodies_.back().get());
        return static_cast<int>(bodies_.size()) - 1;
    }

    void integrateForces(float h) noexcept {
        for (auto& bp : bodies_) {
            Body& bref = *bp;
            if (bref.isDynamic() && !bref.isSleeping) {
                bref.vel += (gravity + bref.force * bref.invMass) * h;
                bref.angVel += bref.torque * bref.invInertia * h;
                float linScale = 1.f / (1.f + h * bref.linearDamping);
                float angScale = 1.f / (1.f + h * bref.angularDamping);
                bref.vel = bref.vel * linScale;
                bref.angVel = bref.angVel * angScale;
                if (bref.fixedRotation) {
                    bref.angVel = 0.f;
                }
            }
        }
    }

    // CCD swept (time-of-impact): Circle contra Circle y AABB sin rotar
    // con el camino exacto original; cualquier otra forma (AABB, poligono,
    // capsula, segmento) barre solo su CENTRO con shapeCastBody(radius=0)
    // contra cualquier otro cuerpo, aproximacion documentada en query.hpp
    // (AABB rotado/poligono/segmento sin engordar; AABB sin rotar y
    // capsula segun su expansion propia). El AABB expandido por el radio
    // del circulo es conservador en las esquinas (slab): el TOI puede
    // adelantarse, pero nunca se tunela en ese camino.
    // Snapshot: si el otro cuerpo se mueve, el TOI se calcula contra su
    // posicion inicial con velocidad relativa, aproximado.
    void sweepFastBodies(float h) {
        swept_.clear();
        if (!(h > 0.f) || bodies_.empty()) {
            return;
        }
        // Escaneo barato: sin candidatos no se toca ningún buffer grande.
        bool anyCandidate = false;
        for (std::size_t scan = 0; scan < ptrs_.size(); ++scan) {
            Body* const scanBody = ptrs_[scan];
            if (!scanBody->isDynamic() || scanBody->isSleeping) {
                continue;
            }
            float const scanRadius = scanBody->boundingRadius();
            if (!(scanRadius > 1e-8f)) {
                continue;
            }
            float const scanDisp = scanBody->vel.length() * h;
            if (!(scanDisp > 1e-8f) || !(scanBody->bullet || scanDisp > scanRadius)) {
                continue;
            }
            anyCandidate = true;
            break;
        }
        if (!anyCandidate) {
            return;
        }
        std::size_t const count = bodies_.size();
        startPos_.resize(count);
        startVel_.resize(count);
        startAngVel_.resize(count);
        for (std::size_t idx = 0; idx < count; ++idx) {
            Body* const bp = ptrs_[idx];
            startPos_[idx] = bp->pos;
            startVel_[idx] = (bp->isStatic() || bp->isSleeping) ? Vec2{} : bp->vel;
            startAngVel_[idx] = (bp->isStatic() || bp->isSleeping) ? 0.f : bp->angVel;
        }
        bestT_.assign(count, 1.f);
        bestN_.resize(count);
        bestOther_.assign(count, -1);
        hasHit_.assign(count, 0);
        swept_.assign(count, 0);
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            Body& bulletBody = *ptrs_[i];
            if (!bulletBody.isDynamic() || bulletBody.isSleeping) {
                continue;
            }
            Circle const* bulletCircle = std::get_if<Circle>(&bulletBody.shape);
            bool const isCircle = bulletCircle != nullptr;
            if (isCircle && !(bulletCircle->radius > 0.f)) {
                continue;
            }
            float const radius = isCircle ? bulletCircle->radius : bulletBody.boundingRadius();
            if (!(radius > 1e-8f)) {
                continue;
            }
            Vec2 const vel = startVel_[i];
            float const disp = vel.length() * h;
            if (!(disp > 1e-8f) || !(bulletBody.bullet || disp > radius)) {
                continue;
            }
            for (std::size_t j = 0; j < bodies_.size(); ++j) {
                if (j == i) {
                    continue;
                }
                Body& otherBody = *ptrs_[j];
                Vec2 const relVel = vel - startVel_[j];
                float const relSpeed = relVel.length();
                if (!(relSpeed > 1e-8f)) {
                    continue;
                }
                float toi = 0.f;
                Vec2 normal{};
                bool found = false;
                if (!isCircle) {
                    // Convexo contra convexo (AABB/polígono): barrido exacto
                    // por diferencia de Minkowski; el resto (cápsula,
                    // segmento) barre el centro con shapeCastBody.
                    auto const convexShape = [](Body const& body) {
                        return std::holds_alternative<AABB>(body.shape) ||
                               std::holds_alternative<ConvexPolygon>(body.shape);
                    };
                    RayHit const castHit =
                        (convexShape(bulletBody) && convexShape(otherBody))
                            ? shapeCastBodyShape(relVel, relSpeed * h, bulletBody, otherBody)
                            : shapeCastBody(startPos_[i], relVel, relSpeed * h, 0.f, otherBody);
                    if (!castHit.hit) {
                        continue;
                    }
                    float const t = castHit.fraction;
                    if (!(t > 1e-6f) || t > 1.f) {
                        continue;
                    }
                    normal = castHit.normal.normalizedOr(-relVel.normalized());
                    toi = t;
                    found = true;
                } else if (Circle const* otherCircle = std::get_if<Circle>(&otherBody.shape)) {
                    Vec2 const d0 = startPos_[i] - startPos_[j];
                    float const sumR = radius + otherCircle->radius;
                    if (d0.length2() <= sumR * sumR) {
                        continue;
                    }
                    float const aCoef = relVel.dot(relVel);
                    float const bCoef = d0.dot(relVel);
                    float const cCoef = d0.length2() - sumR * sumR;
                    float const disc = bCoef * bCoef - aCoef * cCoef;
                    if (disc < 0.f) {
                        continue;
                    }
                    float const tau = (-bCoef - std::sqrt(disc)) / aCoef;
                    float const t = tau / h;
                    if (!(t > 1e-6f) || t > 1.f) {
                        continue;
                    }
                    normal = (d0 + relVel * tau).normalizedOr(-relVel.normalized());
                    toi = t;
                    found = true;
                } else if (isCircle && std::abs(otherBody.angle) <= 1e-6f &&
                           std::holds_alternative<AABB>(otherBody.shape)) {
                    AABB const* otherBox = std::get_if<AABB>(&otherBody.shape);
                    Vec2 const ext{radius, radius};
                    Vec2 const aMin = startPos_[j] + otherBox->lower - ext;
                    Vec2 const aMax = startPos_[j] + otherBox->upper + ext;
                    Vec2 const origin = startPos_[i];
                    if (origin.x >= aMin.x && origin.x <= aMax.x && origin.y >= aMin.y &&
                        origin.y <= aMax.y) {
                        continue;
                    }
                    RayHit const hit = raycastAABB(origin, relVel, relSpeed * h, aMin, aMax);
                    if (!hit.hit) {
                        continue;
                    }
                    float const t = hit.fraction;
                    if (!(t > 1e-6f) || t > 1.f) {
                        continue;
                    }
                    normal = hit.normal.normalizedOr(-relVel.normalized());
                    toi = t;
                    found = true;
                } else {
                    // Caso general: barrido del centro con el radio del
                    // círculo engordando el objetivo cuando la API lo permite
                    // (círculo/cápsula/segmento); para polígonos y AABB
                    // rotado es una aproximación por el centro del movible.
                    float const castRadius = isCircle ? radius : 0.f;
                    RayHit const castHit =
                        shapeCastBody(startPos_[i], relVel, relSpeed * h, castRadius, otherBody);
                    if (!castHit.hit) {
                        continue;
                    }
                    float const t = castHit.fraction;
                    if (!(t > 1e-6f) || t > 1.f) {
                        continue;
                    }
                    normal = castHit.normal.normalizedOr(-relVel.normalized());
                    toi = t;
                    found = true;
                }
                if (!found || relVel.dot(normal) >= 0.f) {
                    continue;
                }
                if (!hasHit_[i] || toi < bestT_[i]) {
                    hasHit_[i] = 1;
                    bestT_[i] = toi;
                    bestN_[i] = normal;
                    bestOther_[i] = static_cast<int>(j);
                }
            }
        }
        for (std::size_t i = 0; i < bodies_.size(); ++i) {
            if (!hasHit_[i] || bestOther_[i] < 0) {
                continue;
            }
            Body& bulletBody = *ptrs_[i];
            Body& otherBody = *ptrs_[static_cast<std::size_t>(bestOther_[i])];
            if (otherBody.isDynamic() && otherBody.isSleeping) {
                otherBody.wake();
            }
            bulletBody.pos = startPos_[i] + startVel_[i] * (h * bestT_[i]);
            bool const isCircle = std::get_if<Circle>(&bulletBody.shape) != nullptr;
            if (!isCircle && !bulletBody.fixedRotation) {
                bulletBody.angle += startAngVel_[i] * (h * bestT_[i]);
            } else if (!isCircle) {
                bulletBody.angVel = 0.f;
            }
            Vec2 const normal = bestN_[i].normalizedOr({1.f, 0.f});
            if (isCircle) {
                Vec2 const relVel =
                    startVel_[i] - startVel_[static_cast<std::size_t>(bestOther_[i])];
                float const relN = relVel.dot(normal);
                if (relN < 0.f) {
                    float const rest =
                        std::clamp(bulletBody.restitution * otherBody.restitution, 0.f, 1.f);
                    float const invOther = otherBody.isDynamic() ? otherBody.invMass : 0.f;
                    float const invSum = bulletBody.invMass + invOther;
                    if (invSum > 1e-12f) {
                        float const impulse = -(1.f + rest) * relN / invSum;
                        bulletBody.vel += normal * (impulse * bulletBody.invMass);
                        otherBody.vel -= normal * (impulse * invOther);
                    } else {
                        bulletBody.vel -= normal * ((1.f + rest) * relN);
                    }
                }
            } else {
                // Punto de contacto "real" (no el del casco de Minkowski):
                // el punto de la superficie del movible más cercano a la
                // dirección del obstáculo respecto del impacto.
                Vec2 const outside = bulletBody.pos - normal * (bulletBody.boundingRadius() + 1.f);
                Vec2 const contactPoint = closestPointOnShape(outside, bulletBody);
                Vec2 const rBullet = contactPoint - bulletBody.pos;
                Vec2 const rOther = contactPoint - otherBody.pos;
                Vec2 const relPointVel =
                    pointVelocity(bulletBody, rBullet) - pointVelocity(otherBody, rOther);
                float const vn = relPointVel.dot(normal);
                if (vn < 0.f) {
                    float const rest =
                        std::clamp(bulletBody.restitution * otherBody.restitution, 0.f, 1.f);
                    float const massNormal =
                        effectiveMass(otherBody, bulletBody, rOther, rBullet, normal);
                    if (massNormal > 0.f) {
                        float const delta = -(1.f + rest) * vn * massNormal;
                        applyPointImpulse(otherBody, bulletBody, normal * delta, rOther, rBullet);
                    }
                }
            }
            // Movimiento restante del substep tras el impacto: conserva
            // (1 - t)·h de desplazamiento con la velocidad post-impulso (los
            // barredos no pasan por integratePositions).
            bulletBody.pos += bulletBody.vel * ((1.f - bestT_[i]) * h);
            if (!bulletBody.fixedRotation) {
                bulletBody.angle += bulletBody.angVel * ((1.f - bestT_[i]) * h);
            }
            swept_[i] = 1;
        }
        return;
    }

    void integratePositions(float h, std::span<const char> swept) noexcept {
        for (std::size_t idx = 0; idx < ptrs_.size(); ++idx) {
            Body& bref = *ptrs_[idx];
            if (bref.isStatic() || bref.isSleeping) {
                continue;
            }
            if (idx < swept.size() && swept[idx] != 0) {
                continue;
            }
            bref.pos += bref.vel * h;
            if (!bref.fixedRotation) {
                bref.angle += bref.angVel * h;
            } else {
                bref.angVel = 0.f;
            }
        }
    }

    void updateSleep(float h) noexcept {
        for (auto& bp : bodies_) {
            Body& bref = *bp;
            if (!bref.isDynamic() || !bref.allowSleep || bref.isSleeping) {
                continue;
            }
            float lv = bref.vel.length();
            float av = bref.angVel >= 0.f ? bref.angVel : -bref.angVel;
            bool slow = lv < sleepVelocity && av < (sleepVelocity > 1e-8f ? sleepVelocity : 1e-8f);
            if (slow) {
                bref.sleepTimer += h;
            } else {
                bref.sleepTimer = 0.f;
            }
            if (bref.sleepTimer >= sleepTime) {
                bref.sleep();
            }
        }
    }

    void buildGrid() {
        gridEntries_.clear();
        float cs = cellSize > 1e-3f ? cellSize : 64.f;
        for (std::size_t i = 0; i < ptrs_.size(); ++i) {
            Body& bref = *ptrs_[i];
            float const r = radii_[i];
            int const x0 = static_cast<int>(std::floor((bref.pos.x - r) / cs));
            int const x1 = static_cast<int>(std::floor((bref.pos.x + r) / cs));
            int const y0 = static_cast<int>(std::floor((bref.pos.y - r) / cs));
            int const y1 = static_cast<int>(std::floor((bref.pos.y + r) / cs));
            int const id = static_cast<int>(i);
            for (int cx = x0; cx <= x1; ++cx) {
                for (int cy = y0; cy <= y1; ++cy) {
                    gridEntries_.push_back(CellEntry{hashCell(cx, cy), id});
                }
            }
        }
        // Ordenar por celda evita el unordered_map: las entradas de una misma
        // celda quedan contiguas y se pueden parear sin listas por celda.
        // Radix LSD estable: al generarse en orden de id creciente, la
        // estabilidad preserva ese orden dentro de cada celda (a<b en pairs).
        if (gridEntries_.size() > 1U) {
            gridEntriesAux_.resize(gridEntries_.size());
            auto* src = &gridEntries_;
            auto* dst = &gridEntriesAux_;
            for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
                radixCount_.fill(0U);
                for (CellEntry const& e : *src) {
                    auto const k = static_cast<std::uint64_t>(e.key);
                    ++radixCount_[static_cast<std::size_t>((k >> shift) & 0xFFU)];
                }
                std::size_t sum = 0U;
                for (std::size_t b = 0U; b < radixCount_.size(); ++b) {
                    std::size_t const c = radixCount_[b];
                    radixCount_[b] = sum;
                    sum += c;
                }
                for (CellEntry const& e : *src) {
                    auto const k = static_cast<std::uint64_t>(e.key);
                    (*dst)[radixCount_[static_cast<std::size_t>((k >> shift) & 0xFFU)]++] = e;
                }
                std::swap(src, dst);
            }
            // El número de pasadas LSD no garantiza que el resultado quede en
            // gridEntries_: se devuelve siempre al contenedor esperado.
            if (src != &gridEntries_) {
                gridEntries_.swap(gridEntriesAux_);
            }
        }
    }

    void pairsSpatialHash() {
        buildGrid();
        pairs_.clear();
        std::size_t start = 0;
        while (start < gridEntries_.size()) {
            std::size_t end = start + 1U;
            while (end < gridEntries_.size() && gridEntries_[end].key == gridEntries_[start].key) {
                ++end;
            }
            for (std::size_t i = start; i < end; ++i) {
                for (std::size_t j = i + 1U; j < end; ++j) {
                    int a = gridEntries_[i].id;
                    int b = gridEntries_[j].id;
                    if (a != b) {
                        pairs_.emplace_back(a, b);
                    }
                }
            }
            start = end;
        }
        // Radix LSD estable sobre la clave combinada; unique() sigue válido.
        if (pairs_.size() > 1U) {
            pairsAux_.resize(pairs_.size());
            auto* src = &pairs_;
            auto* dst = &pairsAux_;
            for (unsigned int shift = 0U; shift < 64U; shift += 8U) {
                radixCount_.fill(0U);
                for (auto const& pr : *src) {
                    auto const k =
                        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(pr.first)) << 32U) |
                        static_cast<std::uint64_t>(static_cast<std::uint32_t>(pr.second));
                    ++radixCount_[static_cast<std::size_t>((k >> shift) & 0xFFU)];
                }
                std::size_t sum = 0U;
                for (std::size_t b = 0U; b < radixCount_.size(); ++b) {
                    std::size_t const c = radixCount_[b];
                    radixCount_[b] = sum;
                    sum += c;
                }
                for (auto const& pr : *src) {
                    auto const k =
                        (static_cast<std::uint64_t>(static_cast<std::uint32_t>(pr.first)) << 32U) |
                        static_cast<std::uint64_t>(static_cast<std::uint32_t>(pr.second));
                    (*dst)[radixCount_[static_cast<std::size_t>((k >> shift) & 0xFFU)]++] = pr;
                }
                std::swap(src, dst);
            }
            // Igual que en buildGrid: el contenedor con el resultado final
            // se deja en pairs_ independientemente de la paridad de swaps.
            if (src != &pairs_) {
                pairs_.swap(pairsAux_);
            }
        }
        pairs_.erase(std::unique(pairs_.begin(), pairs_.end()), pairs_.end());
    }

    void pairsSAP() {
        eps_.clear();
        for (std::size_t i = 0; i < ptrs_.size(); ++i) {
            Body& bref = *ptrs_[i];
            float const r = radii_[i];
            int const id = static_cast<int>(i);
            eps_.push_back(SAPEndpoint{bref.pos.x - r, id, false});
            eps_.push_back(SAPEndpoint{bref.pos.x + r, id, true});
        }
        std::sort(eps_.begin(), eps_.end(), [](SAPEndpoint const& a, SAPEndpoint const& b) {
            if (a.x != b.x) {
                return a.x < b.x;
            }
            if (a.isMax != b.isMax) {
                return !a.isMax;
            }
            return a.id < b.id;
        });
        pairs_.clear();
        active_.clear();
        for (SAPEndpoint const& ep : eps_) {
            if (!ep.isMax) {
                Body& bref = *ptrs_[static_cast<std::size_t>(ep.id)];
                float const r = radii_[static_cast<std::size_t>(ep.id)];
                for (int otherId : active_) {
                    Body& oref = *ptrs_[static_cast<std::size_t>(otherId)];
                    float const orad = radii_[static_cast<std::size_t>(otherId)];
                    float dy = bref.pos.y - oref.pos.y;
                    float ady = dy >= 0.f ? dy : -dy;
                    if (ady <= r + orad) {
                        int a = ep.id < otherId ? ep.id : otherId;
                        int b = ep.id < otherId ? otherId : ep.id;
                        pairs_.emplace_back(a, b);
                    }
                }
                active_.push_back(ep.id);
            } else {
                auto it = std::remove(active_.begin(), active_.end(), ep.id);
                active_.erase(it, active_.end());
            }
        }
        std::sort(pairs_.begin(), pairs_.end());
        pairs_.erase(std::unique(pairs_.begin(), pairs_.end()), pairs_.end());
    }

    void findContacts() {
        contacts_.clear();
        curGeom_.clear();
        contacts_.reserve(pairs_.size());
        bool const havePrev = prevGeom_.size() == prevContacts_.size() && !prevContacts_.empty();
        std::size_t prevIdx = 0;
        for (auto const& pr : pairs_) {
            if (havePrev) {
                while (prevIdx < prevContacts_.size() && (prevContacts_[prevIdx].a < pr.first ||
                                                          (prevContacts_[prevIdx].a == pr.first &&
                                                           prevContacts_[prevIdx].b < pr.second))) {
                    ++prevIdx;
                }
            }
            int const ai = pr.first;
            int const bi = pr.second;
            std::size_t const aIdx = static_cast<std::size_t>(ai);
            std::size_t const bIdx = static_cast<std::size_t>(bi);
            Body& aref = *ptrs_[aIdx];
            Body& bref = *ptrs_[bIdx];
            float const sumR = radii_[aIdx] + radii_[bIdx];
            if ((bref.pos - aref.pos).length2() > sumR * sumR) {
                continue;
            }
            bool reused = false;
            if (havePrev && prevIdx < prevContacts_.size() && prevContacts_[prevIdx].a == ai &&
                prevContacts_[prevIdx].b == bi) {
                ManifoldGeom const& pg = prevGeom_[prevIdx];
                // Epsilon en vez de igualdad bit a bit: el jitter sub-píxel de
                // los cuerpos en reposo no invalida la reutilización.
                auto const posSame = [](Vec2 a, Vec2 b) { return (a - b).length2() < 1e-12f; };
                reused = posSame(pg.posA, aref.pos) && posSame(pg.posB, bref.pos) &&
                         std::fabs(pg.angA - aref.angle) < 1e-6f &&
                         std::fabs(pg.angB - bref.angle) < 1e-6f;
            }
            if (reused) {
                Manifold c = prevContacts_[prevIdx];
                c.restitution = aref.restitution * bref.restitution;
                c.friction = std::sqrt(aref.friction * bref.friction);
                c.relVel = bref.vel - aref.vel;
                contacts_.push_back(c);
                curGeom_.push_back(ManifoldGeom{aref.pos, bref.pos, aref.angle, bref.angle});
            } else {
                Manifold c{};
                if (collide(aref, bref, c)) {
                    c.a = ai;
                    c.b = bi;
                    contacts_.push_back(c);
                    curGeom_.push_back(ManifoldGeom{aref.pos, bref.pos, aref.angle, bref.angle});
                }
            }
        }
    }

    static void applyPointImpulse(Body& aref, Body& bref, Vec2 imp, Vec2 armA, Vec2 armB) noexcept {
        aref.vel -= imp * aref.invMass;
        aref.angVel -= aref.invInertia * armA.cross(imp);
        bref.vel += imp * bref.invMass;
        bref.angVel += bref.invInertia * armB.cross(imp);
    }

    [[nodiscard]] static float effectiveMass(Body const& aref, Body const& bref, Vec2 armA,
                                             Vec2 armB, Vec2 dir) noexcept {
        float const crossA = armA.cross(dir);
        float const crossB = armB.cross(dir);
        float const kn = aref.invMass + bref.invMass + aref.invInertia * crossA * crossA +
                         bref.invInertia * crossB * crossB;
        return kn > 1e-12f ? 1.f / kn : 0.f;
    }

    /// Notifica Begin/End una vez por step (no por substep) comparando los
    /// contactos al principio del step con los del final. Los pares que no
    /// involucran a ningún cuerpo dinámico no generan callbacks.
    void fireContactCallbacks(std::span<Manifold const> startContacts) {
        auto const isDynamicPair = [this](int a, int b) noexcept {
            return ptrs_[static_cast<std::size_t>(a)]->isDynamic() ||
                   ptrs_[static_cast<std::size_t>(b)]->isDynamic();
        };
        prevKeys_.clear();
        prevKeys_.reserve(startContacts.size() * 2U + 1U);
        for (auto const& pc : startContacts) {
            prevKeys_.insert(pairKey(pc.a, pc.b));
        }
        curKeys_.clear();
        curKeys_.reserve(contacts_.size() * 2U + 1U);
        for (auto const& c : contacts_) {
            curKeys_.insert(pairKey(c.a, c.b));
        }
        if (!beginCallbacks_.empty()) {
            auto const cbs = beginCallbacks_; // copia única por step (reentrancia segura)
            for (auto const& c : contacts_) {
                if (!isDynamicPair(c.a, c.b)) {
                    continue;
                }
                if (prevKeys_.find(pairKey(c.a, c.b)) != prevKeys_.end()) {
                    continue;
                }
                for (auto const& cb : cbs) {
                    cb(c);
                }
            }
        }
        if (!endCallbacks_.empty()) {
            auto const cbs = endCallbacks_;
            for (auto const& pc : startContacts) {
                if (!isDynamicPair(pc.a, pc.b)) {
                    continue;
                }
                if (curKeys_.find(pairKey(pc.a, pc.b)) != curKeys_.end()) {
                    continue;
                }
                for (auto const& cb : cbs) {
                    cb(pc);
                }
            }
        }
    }

    void stepOnce(float h) {
        // boundingRadius solo depende de la forma: un std::visit por cuerpo
        // y step basta, no uno por par candidato.
        radii_.resize(ptrs_.size());
        for (std::size_t i = 0; i < ptrs_.size(); ++i) {
            radii_[i] = ptrs_[i]->boundingRadius();
        }
        preForceVel_.resize(ptrs_.size());
        for (std::size_t i = 0; i < ptrs_.size(); ++i) {
            preForceVel_[i] = ptrs_[i]->vel;
        }
        integrateForces(h);
        float maxRadius = 0.f;
        for (float const r : radii_) {
            if (r > maxRadius) {
                maxRadius = r;
            }
        }
        float const cs = cellSize > 1e-3f ? cellSize : 64.f;
        // Un SpatialHash con cuerpos gigantes multiplicaria las entradas de
        // rejilla sin límite: con radios muy por encima de la celda, SAP es
        // el fallback seguro.
        bool const gridExplodes = maxRadius > cs * 16.f;
        if (broadPhaseType == BroadPhaseType::SAP || gridExplodes) {
            pairsSAP();
        } else {
            pairsSpatialHash();
        }
        findContacts();

        for (auto const& c : contacts_) {
            std::size_t const bIdx = static_cast<std::size_t>(c.b);
            std::size_t const aIdx = static_cast<std::size_t>(c.a);
            if (ptrs_[bIdx]->isDynamic() && ptrs_[bIdx]->isSleeping && partnerMovingPre(aIdx)) {
                ptrs_[bIdx]->wake();
            }
            if (ptrs_[aIdx]->isDynamic() && ptrs_[aIdx]->isSleeping && partnerMovingPre(bIdx)) {
                ptrs_[aIdx]->wake();
            }
        }

        solver_.clear();
        solver_.reserve(contacts_.size() * 2U);
        for (auto const& c : contacts_) {
            Body& aref = *ptrs_[static_cast<std::size_t>(c.a)];
            Body& bref = *ptrs_[static_cast<std::size_t>(c.b)];
            Vec2 const normal = c.normal.normalizedOr({1.f, 0.f});
            Vec2 const tangent = normal.perpendicular();
            int const pointCount = c.count >= 2 ? 2 : 1;
            Vec2 const points[2] = {c.point, c.point2};
            WarmEntry probe{};
            probe.key = pairKey(c.a, c.b);
            auto wit = std::lower_bound(
                warmEntries_.begin(), warmEntries_.end(), probe,
                [](WarmEntry const& a, WarmEntry const& b) { return a.key < b.key; });
            bool const hasWarm = wit != warmEntries_.end() && wit->key == probe.key;
            for (int pi = 0; pi < pointCount; ++pi) {
                SolverContact sc{};
                sc.a = c.a;
                sc.b = c.b;
                sc.pointIndex = pi;
                sc.pointCount = pointCount;
                sc.point = points[pi];
                sc.normal = normal;
                sc.armA = sc.point - aref.pos;
                sc.armB = sc.point - bref.pos;
                sc.massNormal = effectiveMass(aref, bref, sc.armA, sc.armB, normal);
                sc.massTangent = effectiveMass(aref, bref, sc.armA, sc.armB, tangent);
                sc.friction = c.friction;
                sc.depth = c.depth;
                if (hasWarm) {
                    sc.accumNormal = pi == 0 ? wit->ws.normal1 : wit->ws.normal2;
                    sc.accumTangent = pi == 0 ? wit->ws.tangent1 : wit->ws.tangent2;
                }
                Vec2 const rel0 = pointVelocity(bref, sc.armB) - pointVelocity(aref, sc.armA);
                float const vn0 = rel0.dot(normal);
                float restThresh = gravity.length() * h;
                if (restThresh < 1.f) {
                    restThresh = 1.f;
                }
                float const e = vn0 < -restThresh ? c.restitution : 0.f;
                sc.restitutionBias = -e * vn0;
                solver_.push_back(sc);
            }
        }

        for (auto const& jointPtr : joints_) {
            if (jointPtr->a >= 0 && jointPtr->b >= 0 &&
                static_cast<std::size_t>(jointPtr->a) < bodies_.size() &&
                static_cast<std::size_t>(jointPtr->b) < bodies_.size()) {
                Body& bodyA = body(jointPtr->a);
                Body& bodyB = body(jointPtr->b);
                // Un cuerpo dormido con invMass=0 ignoraría el joint: si su
                // pareja se mueve (estado liquido del substep anterior), hay
                // que despertarlo antes de preparar.
                if (bodyA.isSleeping && partnerMovingPre(static_cast<std::size_t>(jointPtr->b))) {
                    bodyA.wake();
                }
                if (bodyB.isSleeping && partnerMovingPre(static_cast<std::size_t>(jointPtr->a))) {
                    bodyB.wake();
                }
                jointPtr->prepare(bodyA, bodyB, 1.f / h);
            }
        }

        for (auto& sc : solver_) {
            if (sc.accumNormal == 0.f && sc.accumTangent == 0.f) {
                continue;
            }
            Body& aref = *ptrs_[static_cast<std::size_t>(sc.a)];
            Body& bref = *ptrs_[static_cast<std::size_t>(sc.b)];
            Vec2 const tangent = sc.normal.perpendicular();
            Vec2 const imp = sc.normal * sc.accumNormal + tangent * sc.accumTangent;
            applyPointImpulse(aref, bref, imp, sc.armA, sc.armB);
        }

        int const velCount = velocityIterations > 0 ? velocityIterations : 0;
        for (int it = 0; it < velCount; ++it) {
            for (auto& sc : solver_) {
                if (sc.massNormal <= 0.f) {
                    continue;
                }
                Body& aref = *ptrs_[static_cast<std::size_t>(sc.a)];
                Body& bref = *ptrs_[static_cast<std::size_t>(sc.b)];
                Vec2 const tangent = sc.normal.perpendicular();
                Vec2 rel = pointVelocity(bref, sc.armB) - pointVelocity(aref, sc.armA);
                float const vn = rel.dot(sc.normal);
                float delta = -(vn - sc.restitutionBias) * sc.massNormal;
                float const oldAcc = sc.accumNormal;
                float const newAcc = oldAcc + delta > 0.f ? oldAcc + delta : 0.f;
                delta = newAcc - oldAcc;
                sc.accumNormal = newAcc;
                if (delta != 0.f) {
                    applyPointImpulse(aref, bref, sc.normal * delta, sc.armA, sc.armB);
                }
                if (sc.massTangent <= 0.f) {
                    continue;
                }
                rel = pointVelocity(bref, sc.armB) - pointVelocity(aref, sc.armA);
                float const vt = rel.dot(tangent);
                float deltaT = -vt * sc.massTangent;
                float const maxF = sc.friction * sc.accumNormal;
                float const oldT = sc.accumTangent;
                float newT = oldT + deltaT;
                if (newT > maxF) {
                    newT = maxF;
                }
                if (newT < -maxF) {
                    newT = -maxF;
                }
                deltaT = newT - oldT;
                sc.accumTangent = newT;
                if (deltaT != 0.f) {
                    applyPointImpulse(aref, bref, tangent * deltaT, sc.armA, sc.armB);
                }
            }
            for (auto const& jointPtr : joints_) {
                if (jointPtr->a >= 0 && jointPtr->b >= 0 &&
                    static_cast<std::size_t>(jointPtr->a) < bodies_.size() &&
                    static_cast<std::size_t>(jointPtr->b) < bodies_.size()) {
                    jointPtr->solve(body(jointPtr->a), body(jointPtr->b));
                }
            }
        }

        int const posCount = positionIterations > 0 ? positionIterations : 0;
        for (int it = 0; it < posCount; ++it) {
            for (auto& sc : solver_) {
                if (sc.massNormal <= 0.f) {
                    continue;
                }
                float const eff = sc.depth - 0.25f;
                if (eff <= 0.f) {
                    continue;
                }
                // P = corrección * masa efectiva: reparte entre traslación y
                // rotación; con 2 puntos cada uno corrige su mitad.
                float const corrMag = eff * 0.8f / static_cast<float>(sc.pointCount);
                Vec2 const imp = sc.normal * (corrMag * sc.massNormal);
                Body& aref = *ptrs_[static_cast<std::size_t>(sc.a)];
                Body& bref = *ptrs_[static_cast<std::size_t>(sc.b)];
                aref.pos -= imp * aref.invMass;
                aref.angle -= aref.invInertia * sc.armA.cross(imp);
                bref.pos += imp * bref.invMass;
                bref.angle += bref.invInertia * sc.armB.cross(imp);
                sc.depth -= corrMag * static_cast<float>(sc.pointCount);
                if (sc.depth < 0.f) {
                    sc.depth = 0.f;
                }
            }
        }

        sweepFastBodies(h);
        integratePositions(h, std::span<const char>(swept_.data(), swept_.size()));
        updateSleep(h);

        // solver_ agrupa los puntos de un contacto seguidos y pairs_ va
        // ordenado, así que las claves llegan no decrecientes: basta mirar
        // la última entrada para fusionar los dos puntos de un mismo par.
        warmEntries_.clear();
        warmEntries_.reserve(solver_.size());
        for (auto const& sc : solver_) {
            long long const key = pairKey(sc.a, sc.b);
            if (warmEntries_.empty() || warmEntries_.back().key != key) {
                WarmEntry entry{};
                entry.key = key;
                warmEntries_.push_back(entry);
            }
            WarmStart& entry = warmEntries_.back().ws;
            if (sc.pointIndex == 0) {
                entry.normal1 = sc.accumNormal;
                entry.tangent1 = sc.accumTangent;
            } else {
                entry.normal2 = sc.accumNormal;
                entry.tangent2 = sc.accumTangent;
            }
        }
        prevContacts_ = contacts_;
        prevGeom_ = curGeom_;
    }
};

} // namespace phys2d
