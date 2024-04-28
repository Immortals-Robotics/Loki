#pragma once
#include "../utility/vector_helper.h"

enum TeamColor
{
    TEAM_COLOR_BLUE   = 0,
    TEAM_COLOR_YELLOW = 1,
};
class VisualizationRenderer
{
private:
    const float ballRadius;
    const float robotRadius;
    const float robotArcAngle;
    ImVec2      wSize;
    ImVec2      overallFieldSize;
    float       zoomScale;
    float       upScalingFactor;
    Font        visualizationFont;
    void        CalculateZoom();
    Vector2     ConvertSignedVecToPixelVec(ImVec2 _signedVec);
    int         ConvertRealityUnitToPixels(float _value);

public:
    Shader fxaaShader;

    VisualizationRenderer(ImVec2 _wSize, float _upScalingFactor);
    RenderTexture visualizaionTexture, shaderVisualizationTexture;
    void          init(void);
    void          DrawRectVec(ImVec2 _v1, ImVec2 _v2, Color _color, bool _isFilled, float _thickness = 1,
                              unsigned char transparency = 255);
    void          DrawCircleVec(ImVec2 _center, float _rad, Color _color, bool _isFilled, float _thickness = 1,
                                unsigned char _transparency = 255);
    void          DrawCircleSectorVec(ImVec2 _center, float _rad, Color _color, float _startAngle, float _endAngle,
                                      bool _isFilled, unsigned char _transparency = 255);
    void DrawLineVec(ImVec2 _v1, ImVec2 _v2, Color _color, float _thickness = 1, unsigned char _transparency = 255);
    void DrawTextVec(ImVec2 _pos, std::string _str, int _fontSize = 12, Color _color = WHITE,
                     unsigned char _transparency = 255);

    void DrawField(const Protos::SSL_GeometryFieldSize &data);
    void DrawRobots(const google::protobuf::RepeatedPtrField<Protos::SSL_DetectionRobot> &data, TeamColor color);
    void DrawBalls(const google::protobuf::RepeatedPtrField<Protos::SSL_DetectionBall> &data);
    void ApplyShader(void);
};
