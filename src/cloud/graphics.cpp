#include "accelerators/cloud.h"
#include "cloud/manager.h"
#include "core/camera.h"
#include "core/geometry.h"
#include "core/sampler.h"
#include "core/stats.h"
#include "integrators/cloud.h"
#include "messages/utils.h"
#include "pbrt/main.h"
#include "pbrt/raystate.h"



using namespace std;

namespace pbrt {

// STAT_COUNTER("Integrator/Camera rays generated", nCameraRays);
// STAT_COUNTER("Integrator/Total rays traced", totalRays);
// STAT_COUNTER("Intersections/Regular ray intersection tests",
//              nIntersectionTests);

namespace scene {

string GetObjectName(const ObjectType type, const uint32_t id) {
    return SceneManager::getFileName(type, id);
}

Base::Base() {}

Base::~Base() {}

Base::Base(Base &&) = default;
Base &Base::operator=(Base &&) = default;

Base::Base(const std::string &path, const int samplesPerPixel) {
    using namespace pbrt::global;

    PbrtOptions.nThreads = 1;

    manager->init(path);

    auto reader = manager->GetReader(ObjectType::Camera);
    protobuf::Camera proto_camera;
    reader->read(&proto_camera);
    camera = camera::from_protobuf(proto_camera, transformCache);

    reader = manager->GetReader(ObjectType::Sampler);
    protobuf::Sampler proto_sampler;
    reader->read(&proto_sampler);
    sampler = sampler::from_protobuf(proto_sampler, samplesPerPixel);

    reader = manager->GetReader(ObjectType::Lights);
    while (!reader->eof()) {
        protobuf::Light proto_light;
        reader->read(&proto_light);
        lights.push_back(move(light::from_protobuf(proto_light)));
    }

    reader = manager->GetReader(ObjectType::Scene);
    protobuf::Scene proto_scene;
    reader->read(&proto_scene);
    fakeScene = make_unique<Scene>(from_protobuf(proto_scene));

    for (auto &light : lights) {
        light->Preprocess(*fakeScene);
    }

    printf("Loading treelet count\n");
    const auto treeletCount = manager->treeletCount();
    treeletDependencies.resize(treeletCount);

    for (TreeletId i = 0; i < treeletCount; i++) {
        treeletDependencies[i] = manager->getTreeletDependencies(i);
    }

    printf("Loaded manifest\n");

    this->samplesPerPixel = sampler->samplesPerPixel;
    sampleBounds = camera->film->GetSampleBounds();
    sampleExtent = sampleBounds.Diagonal();
    totalPaths = sampleBounds.Area() * sampler->samplesPerPixel;
}

Base LoadBase(const std::string &path, const int samplesPerPixel) {
    return {path, samplesPerPixel};
}

Base::Base(char* buffer, uint64_t size, const int samplesPerPixel) {
    printf("In graphics loading base\n");
    using namespace pbrt::global;

    PbrtOptions.nThreads = 1;

    // manager->init(path);
    char* buf_now = buffer;
    uint32_t next_size = 0;
    memcpy(&next_size, buf_now, sizeof(uint32_t));
    printf("finished first memory read\n");
    buf_now += sizeof(uint32_t);
    protobuf::Camera proto_camera;
    bool success = proto_camera.ParseFromArray(buf_now, next_size);
    CHECK_EQ(success, true);
    buf_now += next_size;
    printf("parsed raw protobuf\n");
    camera = camera::from_protobuf(proto_camera, transformCache);
    printf("ran from protobuf\n");

    next_size = 0;
    memcpy(&next_size, buf_now, sizeof(uint32_t));
    buf_now += sizeof(uint32_t);
    protobuf::Sampler proto_sampler;
    success = proto_sampler.ParseFromArray(buf_now, next_size);
    CHECK_EQ(success, true);
    buf_now += next_size;
    sampler = sampler::from_protobuf(proto_sampler, samplesPerPixel);

    printf("parsed sampler\n");

    uint32_t num_lights = 0;
    memcpy(&num_lights, buf_now, sizeof(uint32_t));
    buf_now += sizeof(uint32_t);
    for (int i = 0; i < num_lights; i++) {
        next_size = 0;
        memcpy(&next_size, buf_now, sizeof(uint32_t));
        buf_now += sizeof(uint32_t);
        protobuf::Light proto_light;
        success = proto_light.ParseFromArray(buf_now, next_size);
        CHECK_EQ(success, true);
        buf_now += next_size;
        lights.push_back(move(light::from_protobuf(proto_light)));
    }

    printf("parsed lights\n");

    next_size = 0;
    memcpy(&next_size, buf_now, sizeof(uint32_t));
    buf_now += sizeof(uint32_t);
    protobuf::Scene proto_scene;
    success = proto_scene.ParseFromArray(buf_now, next_size);
    CHECK_EQ(success, true);
    buf_now += next_size;
    fakeScene = make_unique<Scene>(from_protobuf(proto_scene));

    printf("parsed scene\n");

    for (auto &light : lights) {
        light->Preprocess(*fakeScene);
    }

    printf("ran preprocess\n");

    manager->useNetwork(&buf_now); // Extra layer of indirection allows the manager to update buf_now in case we want to use it again later.
    const auto treeletCount = manager->treeletCount();
    treeletDependencies.resize(treeletCount);

    for (TreeletId i = 0; i < treeletCount; i++) {
        treeletDependencies[i] = manager->getTreeletDependencies(i);
    }

    this->samplesPerPixel = sampler->samplesPerPixel;
    sampleBounds = camera->film->GetSampleBounds();
    sampleExtent = sampleBounds.Diagonal();
    totalPaths = sampleBounds.Area() * sampler->samplesPerPixel;
}

Base LoadNetworkBase(char* buffer, uint64_t size, const int samplesPerPixel) {
    return {buffer, size, samplesPerPixel};
}

void SerializeBaseToBuffer(string camera_filename, string lights_filename, string sampler_filename,
                           string scene_filename, string manifest_filename, char** buffer, uint64_t* size) {
    ostringstream raw_data_out;
    protobuf::RecordReader camera_reader(camera_filename);
    protobuf::Camera camera;
    bool success = camera_reader.read(&camera);
    CHECK_EQ(success, true);
    string cam_out_str;
    camera.SerializeToString(&cam_out_str);
    uint32_t next_size = cam_out_str.length();
    raw_data_out.write((char*)&next_size, sizeof(uint32_t));
    camera.SerializeToOstream(&raw_data_out);

    protobuf::RecordReader sampler_reader(sampler_filename);
    protobuf::Sampler sampler;
    success = sampler_reader.read(&sampler);
    CHECK_EQ(success, true);
    string sam_out_str;
    sampler.SerializeToString(&sam_out_str);
    next_size = sam_out_str.length();
    raw_data_out.write((char*)&next_size, sizeof(uint32_t));
    sampler.SerializeToOstream(&raw_data_out);

    vector<protobuf::Light> all_lights;
    protobuf::RecordReader lights_reader(lights_filename);
    while (!lights_reader.eof()) {
        protobuf::Light proto_light;
        success = lights_reader.read(&proto_light);
        CHECK_EQ(success, true);
        all_lights.emplace_back(move(proto_light));
    }
    uint32_t num_lights = all_lights.size();
    raw_data_out.write((char*)&num_lights, sizeof(uint32_t));
    for (const auto& light : all_lights) {
        string light_out_str;
        light.SerializeToString(&light_out_str);
        next_size = light_out_str.length();
        raw_data_out.write((char*)&next_size, sizeof(uint32_t));
        light.SerializeToOstream(&raw_data_out);
    }

    protobuf::RecordReader scene_reader(scene_filename);
    protobuf::Scene scene;
    success = scene_reader.read(&scene);
    CHECK_EQ(success, true);
    string scene_out_str;
    scene.SerializeToString(&scene_out_str);
    next_size = scene_out_str.length();
    raw_data_out.write((char*)&next_size, sizeof(uint32_t));
    scene.SerializeToOstream(&raw_data_out);

    protobuf::RecordReader manifest_reader(manifest_filename);
    protobuf::Manifest manifest;
    success = manifest_reader.read(&manifest);
    CHECK_EQ(success, true);
    string manifest_out_str;
    manifest.SerializeToString(&manifest_out_str);
    next_size = manifest_out_str.length();
    raw_data_out.write((char*)&next_size, sizeof(uint32_t));
    manifest.SerializeToOstream(&raw_data_out);

    string raw_data_str = raw_data_out.str();
    *buffer = new char[raw_data_str.length()];
    memcpy(*buffer, raw_data_str.c_str(), raw_data_str.length());
    *size = raw_data_str.length();
}

shared_ptr<CloudBVH> LoadTreelet(const string &path, const TreeletId treeletId,
                                 istream *stream) {
    using namespace pbrt::global;
    manager->init(path);
    shared_ptr<CloudBVH> treelet = make_shared<CloudBVH>(treeletId);
    treelet->LoadTreelet(treeletId, stream);
    return treelet;
}

shared_ptr<CloudBVH> LoadNetworkTreelet(const TreeletId treeletId,
                                 char* buffer, uint64_t size) {
    using namespace pbrt::global;
    // manager->init(path);
    printf("attempting to load network treelet\n");
    shared_ptr<CloudBVH> treelet = make_shared<CloudBVH>(treeletId);
    treelet->LoadNetworkTreelet(treeletId, buffer, size);
    return treelet;
}

// TODO: It's kind of silly that this is necessary, although having it on hand does make it very clear exactly which protobufs are
// being serialized for the network messages and in what order.
void SerializeTreeletToBuffer(string treelet_filename, vector<string>& mat_filenames, char** buffer, uint64_t* size) {
    ostringstream raw_data_out;
    uint32_t num_mats = mat_filenames.size();
    raw_data_out.write((char*)&num_mats, sizeof(uint32_t));
    for (int i = 0; i < mat_filenames.size(); i++) {
        protobuf::RecordReader mat_reader(mat_filenames[i]);
        uint32_t mat_id = atoi(mat_filenames[i].substr(mat_filenames[i].find("MAT")).c_str() + strlen("MAT")); // TODO: This will probably break if MAT is somewhere earlier in the path. Fix this.
        protobuf::Material mat;
        bool success = mat_reader.read(&mat);
        CHECK_EQ(success, true);
        string mat_out_str;
        mat.SerializeToString(&mat_out_str);
        uint32_t next_size = mat_out_str.length();
        raw_data_out.write((char*)&mat_id, sizeof(uint32_t));
        raw_data_out.write((char*)&next_size, sizeof(uint32_t));
        mat.SerializeToOstream(&raw_data_out);
    }
    protobuf::RecordReader treelet_reader(treelet_filename);
    uint32_t num_triangle_meshes = 0;
    treelet_reader.read(&num_triangle_meshes);
    raw_data_out.write((char*)&num_triangle_meshes, sizeof(uint32_t));
    for (int i = 0; i < num_triangle_meshes; ++i) {
        protobuf::TriangleMesh tm;
        bool success = treelet_reader.read(&tm);
        CHECK_EQ(success, true);
        string tm_str;
        tm.SerializeToString(&tm_str);
        uint32_t next_size = tm_str.length();
        raw_data_out.write((char*)&next_size, sizeof(uint32_t));
        tm.SerializeToOstream(&raw_data_out);
        printf("written id is %d\n", tm.id());
    }

    while (!treelet_reader.eof()) {
        protobuf::BVHNode proto_node;
        bool success = treelet_reader.read(&proto_node);
        CHECK_EQ(success, true);
        string proto_node_str;
        proto_node.SerializeToString(&proto_node_str);
        uint32_t next_size = proto_node_str.length();
        raw_data_out.write((char*)&next_size, sizeof(uint32_t));
        proto_node.SerializeToOstream(&raw_data_out);
    }
    string raw_data_str = raw_data_out.str();
    *buffer = new char[raw_data_str.length()];
    memcpy(*buffer, raw_data_str.c_str(), raw_data_str.length());
    *size = raw_data_str.length();
}

}  // namespace scene

namespace graphics {

RayStatePtr TraceRay(RayStatePtr &&rayState, const CloudBVH &treelet) {
    return CloudIntegrator::Trace(move(rayState), treelet);
}

pair<RayStatePtr, RayStatePtr> ShadeRay(RayStatePtr &&rayState,
                                        const CloudBVH &treelet,
                                        const vector<shared_ptr<Light>> &lights,
                                        const Vector2i &sampleExtent,
                                        shared_ptr<GlobalSampler> &sampler,
                                        int maxPathDepth, MemoryArena &arena) {
    return CloudIntegrator::Shade(move(rayState), treelet, lights, sampleExtent,
                                  sampler, maxPathDepth, arena);
}

RayStatePtr GenerateCameraRay(const shared_ptr<Camera> &camera,
                              const Point2i &pixel, const uint32_t sample,
                              const uint8_t maxDepth,
                              const Vector2i &sampleExtent,
                              shared_ptr<GlobalSampler> &sampler) {
    const auto samplesPerPixel = sampler->samplesPerPixel;
    const Float rayScale = 1 / sqrt((Float)samplesPerPixel);

    sampler->StartPixel(pixel);
    sampler->SetSampleNumber(sample);

    CameraSample cameraSample = sampler->GetCameraSample(pixel);

    RayStatePtr statePtr = RayState::Create();
    RayState &state = *statePtr;

    state.sample.id =
        (pixel.x + pixel.y * sampleExtent.x) * samplesPerPixel + sample;
    state.sample.dim = sampler->GetCurrentDimension();
    state.sample.pFilm = cameraSample.pFilm;
    state.sample.weight =
        camera->GenerateRayDifferential(cameraSample, &state.ray);
    state.ray.ScaleDifferentials(rayScale);
    state.remainingBounces = maxDepth - 1;
    state.StartTrace();

    //++nCameraRays;
    //++nIntersectionTests;
    //++totalRays;

    return statePtr;
}

void AccumulateImage(const shared_ptr<Camera> &camera,
                     const vector<Sample> &rays) {
    const Bounds2i sampleBounds = camera->film->GetSampleBounds();
    unique_ptr<FilmTile> filmTile = camera->film->GetFilmTile(sampleBounds);

    for (const auto &ray : rays) {
        filmTile->AddSample(ray.pFilm, ray.L, ray.weight, true);
    }

    camera->film->MergeFilmTile(move(filmTile));
}

void WriteImage(const shared_ptr<Camera> &camera) {
    camera->film->WriteImage();
}

}  // namespace graphics

}  // namespace pbrt
