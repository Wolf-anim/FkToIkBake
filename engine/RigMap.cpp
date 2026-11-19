#include "RigMap.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace fk2ik {

namespace {

std::string trim(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::string lower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool toDouble(const std::string& s, double& v) {
    char* end = nullptr;
    v = std::strtod(s.c_str(), &end);
    return end && *end == '\0' && !s.empty();
}

}  // namespace

LimbSettings LimbDefinition::settings() const {
    LimbSettings s = LimbSettings::defaultsFor(type);
    s.mode = mode;
    if (poleDistance > 0) s.poleDistance = poleDistance;
    if (straightLow >= 0) s.straightLow = straightLow;
    if (straightHigh >= 0) s.straightHigh = straightHigh;
    return s;
}

std::string LimbDefinition::validate() const {
    const std::pair<const char*, const std::string*> req[] = {
        {"fkUpper", &fkUpper}, {"fkMid", &fkMid}, {"fkEnd", &fkEnd},
        {"ikUpper", &ikUpper}, {"ikMid", &ikMid}, {"ikEnd", &ikEnd},
        {"ikControl", &ikControl}, {"poleControl", &poleControl}};
    std::string missing;
    for (auto& r : req)
        if (r.second->empty()) missing += std::string(missing.empty() ? "" : ", ") + r.first;
    if (!missing.empty()) return "limb '" + name + "' is missing: " + missing;
    if (!switchAttr.empty() && switchAttr.find('.') == std::string::npos)
        return "limb '" + name + "': switchAttr must be node.attribute";
    return {};
}

const LimbDefinition* RigMap::find(const std::string& n) const {
    for (auto& l : limbs)
        if (l.name == n) return &l;
    return nullptr;
}

bool parseRigMap(const std::string& text, RigMap& out, std::string& error) {
    out.limbs.clear();
    std::istringstream in(text);
    std::string line;
    int lineNo = 0;
    LimbDefinition* cur = nullptr;
    while (std::getline(in, line)) {
        ++lineNo;
        const size_t cpos = line.find_first_of(";#");
        if (cpos != std::string::npos) line = line.substr(0, cpos);
        line = trim(line);
        if (line.empty()) continue;
        auto fail = [&](const std::string& msg) {
            error = "rig map line " + std::to_string(lineNo) + ": " + msg;
            return false;
        };
        if (line.front() == '[') {
            if (line.back() != ']') return fail("unterminated section header");
            LimbDefinition d;
            d.name = trim(line.substr(1, line.size() - 2));
            if (d.name.empty()) return fail("empty limb name");
            if (out.find(d.name)) return fail("duplicate limb '" + d.name + "'");
            out.limbs.push_back(d);
            cur = &out.limbs.back();
            continue;
        }
        const size_t eq = line.find('=');
        if (eq == std::string::npos) return fail("expected key = value");
        if (!cur) return fail("key outside of a [limb] section");
        const std::string key = lower(trim(line.substr(0, eq)));
        const std::string val = trim(line.substr(eq + 1));
        double num = 0;
        if (key == "type") {
            const std::string v = lower(val);
            if (v == "arm") cur->type = LimbType::Arm;
            else if (v == "leg") cur->type = LimbType::Leg;
            else return fail("type must be arm or leg");
        } else if (key == "mode") {
            const std::string v = lower(val);
            if (v == "position") cur->mode = MatchMode::Position;
            else if (v == "rotation" || v == "mocap" || v == "motioncapture") cur->mode = MatchMode::Rotation;
            else return fail("mode must be position or rotation");
        } else if (key == "fkupper") cur->fkUpper = val;
        else if (key == "fkmid") cur->fkMid = val;
        else if (key == "fkend") cur->fkEnd = val;
        else if (key == "ikupper") cur->ikUpper = val;
        else if (key == "ikmid") cur->ikMid = val;
        else if (key == "ikend") cur->ikEnd = val;
        else if (key == "ikcontrol") cur->ikControl = val;
        else if (key == "polecontrol") cur->poleControl = val;
        else if (key == "switchattr") cur->switchAttr = val;
        else if (key == "switchfk" || key == "switchik" || key == "poledistance" ||
                 key == "straightlow" || key == "straighthigh") {
            if (!toDouble(val, num)) return fail("'" + key + "' needs a number");
            if (key == "switchfk") { cur->switchFk = num; cur->hasSwitchValues = true; }
            else if (key == "switchik") { cur->switchIk = num; cur->hasSwitchValues = true; }
            else if (key == "poledistance") cur->poleDistance = num;
            else if (key == "straightlow") cur->straightLow = num;
            else cur->straightHigh = num;
        } else {
            return fail("unknown key '" + key + "'");
        }
    }
    for (auto& l : out.limbs) {
        const std::string v = l.validate();
        if (!v.empty()) { error = v; return false; }
    }
    if (out.limbs.empty()) { error = "rig map contains no limbs"; return false; }
    return true;
}

bool loadRigMap(const std::string& path, RigMap& out, std::string& error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) { error = "cannot open rig map: " + path; return false; }
    std::stringstream ss;
    ss << f.rdbuf();
    return parseRigMap(ss.str(), out, error);
}

}  // namespace fk2ik
