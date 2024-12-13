#ifndef SCENE_TRANSFORMATION_DATA_PLAYER_H
#define SCENE_TRANSFORMATION_DATA_PLAYER_H

#include "TrackerBase.h"
#include "SceneTransformationDataRecorder.h"
#include <Rcs_macros.h>
#include <Rcs_body.h>
#include <Rcs_timer.h>
#include <Rcs_graph.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <fstream>
#include <mutex>
#include <vector>
#include "ComponentBase.h"
#include "ActionScene.h"

namespace aff {

/*! \brief Component to replay recorded transformations on a graph.
 *
 *  The SceneTransformationDataPlayer allows for loading transformations from
 *  a file, filtering out specific bodies (like robot bodies), and applying
 *  these transformations to an RcsGraph during playback. It integrates into
 *  the scene update system by subscribing to the "PostUpdateGraph" event.
 */
class SceneTransformationDataPlayer : public ComponentBase
{
public:
    /*! \brief Constructs the SceneTransformationDataPlayer component.
     *
     *  \param[in] parent Entity class responsible for event subscriptions.
     * 
     */
    SceneTransformationDataPlayer(EntityBase* parent);

    /*! \brief Destructor for the SceneTransformationDataPlayer. */
    ~SceneTransformationDataPlayer();


    /*! \brief Loads recorded transformations from a JSON file.
     *
     *  Reads a file containing transformation data and stores it for playback.
     *  Filters out transformations involving excluded bodies.
     *
     *  \param[in] fileName Path to the JSON file containing transformations.
     */
    void loadFromFile(const std::string& fileName);

    /*! \brief Starts playback from the beginning of the recorded data. */
    void startPlayback();

    /*! \brief Parses a JSON object containing transformation records.
     *
     *  Extracts transformations and timestamps from a JSON object. Filters
     *  out transformations for excluded bodies.
     *
     *  \param[in] json JSON object containing transformation records.
     */
    void parse(const nlohmann::json& json);

    /*! \brief Parses a single transformation entry from a JSON object.
     *
     *  Extracts the relative transformation, child body, and parent body from
     *  the JSON object.
     *
     *  \param[in] transformation JSON object containing a single transformation entry.
     *  \return A BodyTransformation object representing the parsed data.
     */
    BodyTransformation parseTransformation(const nlohmann::json& transformation);

    /*! \brief Identifies all robot bodies in the given graph.
     *
     *  Traverses the graph to find the root robot body and its child bodies,
     *  storing their names for later filtering during playback.
     *
     *  \param[in] graph Pointer to the RcsGraph containing the robot structure.
     */
    void getRobotBodies(RcsGraph* graph);

private:

    /*! \brief Handles the "PostUpdateGraph" event during playback.
     *
     *  Updates the graph with the next transformation record.
     *
     *  \param[in] desired Desired state of the graph.
     *  \param[in] current Current state of the graph.
     */
    void onPostUpdateGraph(RcsGraph* desired, RcsGraph* curent);
    
    /*! \brief Applies the current transformation record to the graph.
     *
     *  Updates the graph with the transformation data at the current playback
     *  index. Increments the playback index after applying transformations.
     *
     *  \param[in] graph Pointer to the RcsGraph being updated.
     */
    
    void updateGraph(RcsGraph* graph);
    

    /*! \brief Applies all transformations in a record to the graph.
     *
     *  Iterates through all transformations in a record and applies them to
     *  their corresponding bodies in the graph.
     *
     *  \param[in] graph Pointer to the RcsGraph being updated.
     *  \param[in] record TransformationRecord containing transformations to apply.
     */
    void applyTransformationToGraph(RcsGraph* graph, const TransformationRecord& record);

    std::vector<TransformationRecord> recordedTransformations; // List of recorded transformations for playback.
    size_t currentIndex; // Index of the current transformation record being applied.
    bool newDataAvailable; // Flag indicating if new transformation data has been loaded.
    std::mutex dataMutex; // Mutex for synchronizing access to recorded transformations.
    double previousTime; // Timestamp of the last transformation application.
    bool playData; // Flag indicating if playback is currently active.
    
    std::vector<std::string> robotBodies; // List of body names to exclude from transformations.

};

} // namespace aff

#endif // SCENE_TRANSFORMATION_DATA_PLAYER_H
