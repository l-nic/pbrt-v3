#include "cloud/lambda-master.h"

#include <chrono>

#include "execution/meow/message.h"
#include "messages/utils.h"

using namespace std;
using namespace std::chrono;
using namespace pbrt;
using namespace meow;
using namespace PollerShortNames;

using OpCode = Message::OpCode;

ResultType LambdaMaster::handleMessages() {
    deque<pair<WorkerId, Message>> unprocessedMessages;

    while (!incomingMessages.empty()) {
        auto front = move(incomingMessages.front());
        incomingMessages.pop_front();

        if (!processMessage(front.first, front.second)) {
            unprocessedMessages.push_back(move(front));
        }
    }

    swap(unprocessedMessages, incomingMessages);
    return ResultType::Continue;
}

bool LambdaMaster::processMessage(const uint64_t workerId,
                                  const meow::Message &message) {
    /* cerr << "[msg:" << Message::OPCODE_NAMES[to_underlying(message.opcode())]
         << "] from worker " << workerId << endl; */

    auto &worker = workers.at(workerId);

    switch (message.opcode()) {
    case OpCode::Hey: {
        worker.aws.logStream = message.payload();

        protobuf::Hey heyProto;
        heyProto.set_worker_id(workerId);
        heyProto.set_job_id(jobId);
        Message msg{0, OpCode::Hey, protoutil::to_string(heyProto)};
        worker.connection->enqueue_write(msg.str());

        if (!worker.initialized) {
            worker.initialized = true;
            initializedWorkers++;
        }

        break;
    }

    case OpCode::WorkerStats: {
        protobuf::WorkerStats proto;
        protoutil::from_string(message.payload(), proto);
        auto stats = from_protobuf(proto);

        if (stats.finishedRays() != 0) {
            lastFinishedRay = lastActionTime = now();
        }

        /* merge into global worker stats */
        workerStats.merge(stats);

        /* merge into local worker stats */
        auto &worker = workers.at(workerId);
        worker.stats.merge(stats);

        if (config.workerStatsInterval > 0 &&
            worker.nextStatusLogTimestamp < proto.timestamp_us()) {
            if (worker.nextStatusLogTimestamp == 0) {
                statsOstream << "start " << worker.id << ' '
                             << proto.worker_start_us() << '\n';
            }

            statsOstream << worker.id << ' ' << proto.timestamp_us() << ' '
                         << protoutil::to_json(to_protobuf(worker.stats))
                         << '\n';

            worker.nextStatusLogTimestamp =
                duration_cast<microseconds>(workerStatsInterval).count() +
                proto.timestamp_us();
        }

        /* if (canSendTiles && cameraRaysRemaining() &&
            stats.queueStats.pending + stats.queueStats.out +
                    stats.queueStats.ray <
                config.newTileThreshold) {
            sendWorkerTile(worker);
        } */

        break;
    }

    case OpCode::FinishedRays: {
        protobuf::RecordReader finishedReader{istringstream(message.payload())};

        while (!finishedReader.eof()) {
            protobuf::FinishedRay proto;
            if (finishedReader.read(&proto)) {
                filmTile->AddSample(from_protobuf(proto.p_film()),
                                    from_protobuf(proto.l()), proto.weight());
            }
        }

        break;
    }

    case OpCode::FinishedPaths: {
        Chunk chunk{message.payload()};

        while (chunk.size()) {
            finishedPathIds.insert(chunk.be64());
            chunk = chunk(8);
        }

        break;
    }

    case OpCode::RayBagEnqueued: {
        protobuf::RayBagEnqueued proto;
        protoutil::from_string(message.payload(), proto);

        for (const auto &item : proto.ray_bags()) {
            queuedRayBags[item.treelet_id()].push(
                {item.treelet_id(), item.bag_id(), item.size()});

            queueSize[item.treelet_id()] += item.size();
        }

        break;
    }

    default:
        throw runtime_error("unhandled message opcode: " +
                            to_string(to_underlying(message.opcode())));
    }

    return true;
}